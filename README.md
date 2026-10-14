

**Raspberry Pi 4 + ROS 2 Humble + slam_toolbox + RViz on a laptop over WiFi**

Version 3.0 — combines the FastDDS-unicast build guide, the Discovery-Server debugging notes, and the calibrated teleoperation/odometry bring-up report. Read the **"Which networking method should I use"** callout in §1 before copying networking commands, and read §6's superseded-version note before copying `base_driver.py` — two incompatible driver/firmware pairs exist across the source material, and only one is current.

---

## 0. What you are building

A differential-drive robot that reads a rotating TFmini (2D laser scan), a BNO085 IMU, and wheel odometry from an ESP32 base; fuses odometry + IMU with an EKF; builds a 2D map with slam_toolbox on the Pi; and shows everything live in RViz on a laptop over WiFi.

### Data flow

```
Sensors / actuators        Raspberry Pi 4 (robot)                Laptop (Docker)
------------------         -----------------------               -------------------------
TFmini (sweep) --UART-->   tfmini_ros_node   -- /scan ----------+
BNO085         --I2C--->   bno085_node      -- /imu/data -----+ |
ESP32 base     --USB--->   base_driver      -- /odom --------+| |
                           ekf_node         -- /odometry/filtered (fuses /odom + /imu/data)
                           robot_state_publisher -- static TF (base_link -> laser_frame ...)
                           slam_toolbox     -- /map  + TF  map -> odom
                                                              ||
                                              WiFi  192.168.1.10  <->  192.168.1.3
                                                              ||
                                              RViz (subscribes /map, /scan, /tf)
                                              teleop_twist_keyboard (/cmd_vel -> base_driver)
```

### Tested configuration

|Item|Value|
|---|---|
|Robot computer|Raspberry Pi 4, Ubuntu 22.04 (64-bit), hostname `raspberrypi`, IP 192.168.1.10|
|Visualization computer|Laptop running a ROS 2 Docker container, IP 192.168.1.3|
|ROS 2 distribution|Humble Hawksbill (Python 3.10)|
|DDS middleware|FastDDS 2.6.x (`rmw_fastrtps_cpp`, the Humble default)|
|SLAM|slam_toolbox, `online_async` mode|
|Laser|TFmini-S, hardware UART `/dev/serial0` @ 115200, on a rotating sweep mount, real range 0.1–12 m|
|IMU|BNO085 via I2C (`/imu/data`)|
|Base controller|ESP32-C3 via USB at `/dev/ttyACM0`, 115200 baud|
|Odometry fusion|`robot_localization` EKF: `/odom` + `/imu/data` -> `/odometry/filtered`|
|URDF frames|base_footprint, base_link, laser_frame, imu_link, camera_link|
|Discovery server|192.168.1.10:11811, GUID prefix `44.53.00.5f.45.50.52.4f.53.49.4d.41` (Discovery-Server method only)|

### Three golden rules (read these first)

1. Every ROS 2 process is its own DDS participant. Every participant must be started in a shell where the required environment variables are set. Exports never "carry over" to new terminals.
2. The `ros2` command-line tools (topic list, echo, rviz2) talk to a background daemon that caches the graph. A stale daemon lies to you. After changing **any** networking setting: `ros2 daemon stop`.
3. A Docker container's filesystem is temporary. Any file you create inside it (XML profiles, RViz configs) is deleted when the container is recreated. Mount files in from the host.

---

## 1. Which networking method should I use?

Two approaches were tried, in this order:

||**A. FastDDS Unicast Peers**|**B. Discovery Server (recommended)**|
|---|---|---|
|How it works|Every participant is given a hardcoded list of peer IP:port combos via an XML profile|One machine (the Pi) runs a lightweight discovery server; everyone else registers as a client|
|Setup complexity|High — hand-built XML, port math, one file per machine|Low — three environment variables, no XML|
|Known failure mode|Python (`rclpy`) nodes are more fragile here; if a node isn't included correctly it silently falls back to multicast|Python nodes silently drop off the server unless `ROS_SUPER_CLIENT=TRUE` is set|
|Status|Documented in Part 2 as the original fix for multicast-blocking WiFi|Documented in Part 7 as the actual root-cause fix after unicast peers still left `rclpy` nodes invisible|

**Use Method B (Discovery Server).** It's what resolved the real debugging session in this project. Method A is kept in Part 2 as reference/fallback — e.g. if you ever need to run without a central server — but do not mix the two: pick one `RMW`/discovery strategy per fleet of terminals, or you'll get the "works via daemon, invisible to rviz2" symptom described in Part 7.

---

## 2. Installation

### 2.1 Raspberry Pi (robot)

Install ROS 2 Humble (headless `ros-base` is enough) and the required packages:

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions python3-rosdep
sudo apt install -y ros-humble-slam-toolbox ros-humble-robot-state-publisher \
  ros-humble-robot-localization ros-humble-nav2-map-server \
  ros-humble-xacro ros-humble-tf2-tools
sudo rosdep init && rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Build the robot workspace (URDF, drivers, launch files, slam params):

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <your robot_base_driver repo>
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Check the slam params file `~/ros2_ws/src/robot_base_driver/config/mapper_params_online_async.yaml` — the laser range limits must match the TFmini, otherwise slam_toolbox prints `minimum/maximum laser range setting exceeds the capabilities of the used Lidar`:

```yaml
min_laser_range: 0.1    # NOT 0.0
max_laser_range: 12.0   # NOT 25.0
```

### 2.2 Laptop (Docker)

Use host networking (ROS 2 + Docker without `--net=host` is a world of pain), mount any DDS profile from the host, and pass the GUI through:

```bash
xhost +local:docker    # once per laptop boot, allows the container to open windows

mkdir -p ~/ros         # host-side home for files that must survive the container

docker run -it --net=host --ipc=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/ros:/root/ros \
  osrf/ros:humble-desktop
```

Inside the container, source ROS on every new shell (or add to `/root/.bashrc`):

```bash
source /opt/ros/humble/setup.bash
```

If you're using Method A (unicast XML), also bind-mount and export `FASTRTPS_DEFAULT_PROFILES_FILE` as shown in Part 3 — skipping the bind mount is the classic cause of `[XMLPARSER Error] Error opening '/root/fastdds_unicast.xml'`, after which FastDDS silently falls back to multicast and nothing works.

---

## 3. Method A — FastDDS Unicast Peers (reference / fallback)

### 3.1 Why this exists

ROS 2 discovers nodes using UDP multicast by default. Many WiFi routers/access points block or mangle multicast (AP isolation), producing this pattern: the robot works perfectly locally, but the laptop sees nothing, some topics, or stale topics. The fix is an "initial peers" list that tells every participant exactly where its peers are. When an initial peers list is defined, FastDDS sends discovery announcements only to those addresses — so the list must include **both** the other machine **and** the local machine itself (loopback), otherwise nodes on the same computer stop seeing each other. (Shared-memory transport does not reliably rescue same-host discovery when `initialPeersList` is used — see `rmw_fastrtps` issue #676.)

### 3.2 The port math

Each participant listens for discovery ("metatraffic") on its own well-known UDP port:

`port = 7400 + 250 * domainID + 10 + 2 * participantID`

With the default domain 0:

|Participant ID on a machine|Metatraffic port|
|---|---|
|0|7410|
|1|7412|
|2|7414|
|3|7416|
|4|7418|
|5|7420|
|6|7422|
|7|7424|

You cannot know in advance which ID each node gets (it depends on startup order, and the `ros2` daemon takes one too), so the peers list simply enumerates IDs 0–7 for each address. Extra entries are harmless.

Do **not** hardcode `<port>7410</port>` in `metatrafficUnicastLocatorList` — that forces every participant on the machine to fight over one port, and only one wins (symptom: `ros2 topic list` shows everything via the daemon, but rviz2/echo see nothing).

### 3.3 XML profiles

Pi version — save as `/home/pi/fastdds_unicast.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <participant profile_name="unicast_participant" is_default_profile="true">
            <rtps>
                <builtin>
                    <metatrafficUnicastLocatorList>
                        <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
                        <locator><udpv4><address>192.168.1.10</address></udpv4></locator>
                    </metatrafficUnicastLocatorList>
                    <initialPeersList>
                        <!-- this machine's own nodes, participant IDs 0-7 -->
                        <locator><udpv4><address>127.0.0.1</address><port>7410</port></udpv4></locator>
                        <locator><udpv4><address>127.0.0.1</address><port>7412</port></udpv4></locator>
                        <locator><udpv4><address>127.0.0.1</address><port>7414</port></udpv4></locator>
                        <locator><udpv4><address>127.0.0.1</address><port>7416</port></udpv4></locator>
                        <locator><udpv4><address>127.0.0.1</address><port>7418</port></udpv4></locator>
                        <locator><udpv4><address>127.0.0.1</address><port>7420</port></udpv4></locator>
                        <locator><udpv4><address>127.0.0.1</address><port>7422</port></udpv4></locator>
                        <locator><udpv4><address>127.0.0.1</address><port>7424</port></udpv4></locator>
                        <!-- laptop, participant IDs 0-7 -->
                        <locator><udpv4><address>192.168.1.3</address><port>7410</port></udpv4></locator>
                        <locator><udpv4><address>192.168.1.3</address><port>7412</port></udpv4></locator>
                        <locator><udpv4><address>192.168.1.3</address><port>7414</port></udpv4></locator>
                        <locator><udpv4><address>192.168.1.3</address><port>7416</port></udpv4></locator>
                        <locator><udpv4><address>192.168.1.3</address><port>7418</port></udpv4></locator>
                        <locator><udpv4><address>192.168.1.3</address><port>7420</port></udpv4></locator>
                        <locator><udpv4><address>192.168.1.3</address><port>7422</port></udpv4></locator>
                        <locator><udpv4><address>192.168.1.3</address><port>7424</port></udpv4></locator>
                    </initialPeersList>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

Laptop version — save as `~/ros/fastdds_unicast.xml` on the host (mounted to `/root/fastdds_unicast.xml` in the container). Identical except: the second metatraffic address is `192.168.1.3`, and the remote peers block (last eight entries) uses `192.168.1.10`. Keep the loopback block — rviz2, the daemon and CLI tools inside the container need it to see each other.

Docker run command for this method:

```bash
docker run -it --net=host --ipc=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/ros/fastdds_unicast.xml:/root/fastdds_unicast.xml \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_unicast.xml \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  osrf/ros:humble-desktop
```

### 3.4 Environment variables (every terminal, both machines)

```bash
# Pi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_unicast.xml

# Laptop (inside Docker)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_unicast.xml
```

Persist in `~/.bashrc` (Pi) and `/root/.bashrc` (container).

Sanity check before launching anything, in every new shell:

```bash
echo $FASTRTPS_DEFAULT_PROFILES_FILE   # must print the path
cat $FASTRTPS_DEFAULT_PROFILES_FILE    # must print the XML (NOT "No such file")
```

---

## 4. Method B — Discovery Server (recommended)

### 4.1 Concept

Instead of every node exchanging discovery traffic with every other node, one lightweight server process (run on the Pi, 192.168.1.10:11811) holds the graph, and every other participant connects to it as a client. This needs far less configuration than Method A and is what actually resolved discovery for this project — see Part 7 for what went wrong with Method A's Python nodes.

### 4.2 Environment variables (every terminal, both machines)

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="192.168.1.10:11811"   # replace with your server IP/port
export ROS_SUPER_CLIENT=TRUE
```

Add to `~/.bashrc` on both machines, then `source ~/.bashrc`.

`ROS_SUPER_CLIENT=TRUE` is not optional for `rclpy` (Python) nodes — see Part 7.1 for why.

### 4.3 Starting the discovery server (Pi)

```bash
fastdds discovery --server-id 0 --ip-address 192.168.1.10 --port 11811
```

Run this in its own dedicated terminal/shell (or as a systemd service) so it isn't accidentally killed with the rest of the stack — see the "isolate terminal environments" rule in Part 7.2.

---

## 5. Workspace & Package Structure

```
~/ros2_ws/
└── src/
    └── robot_base_driver/
        ├── package.xml
        ├── setup.py
        ├── config/
        │   ├── ekf.yaml
        │   └── mapper_params_online_async.yaml
        ├── launch/
        │   └── robot_bringup.launch.py
        └── robot_base_driver/
            ├── __init__.py
            ├── base_driver.py
            └── bno085_node.py
```

### setup.py

```python
from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'robot_base_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Robot Base Driver Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver = robot_base_driver.base_driver:main',
            'bno085_node = robot_base_driver.bno085_node:main',
        ],
    },
)
```

---

## 6. Node Implementation — Firmware, Driver & EKF (canonical, calibrated version)

> **This section supersedes an earlier draft of `base_driver.py`.** An earlier version of this project used a bare `"{pwm_left},{pwm_right}\n"` serial protocol, `wheel_radius=0.032`, `track_width=0.15`, `ticks_per_rev=120.0`, and published `child_frame_id='base_link'`. That version is **not compatible** with the firmware below (different command format) and used uncalibrated kinematic constants. Use the versions in this section; do not mix ESP32 firmware from one version with a driver from the other, or the robot will accept serial writes silently and never move.

### 6.1 Lessons that shaped this version

These were real failures during bring-up, kept here because the fix isn't obvious from the code alone:

|Problem|Root cause|Fix applied below|
|---|---|---|
|TF tree disconnected, EKF wouldn't publish a usable `map → base_footprint` chain|EKF's `base_link_frame` was pointed at `base_link`, but the URDF's actual root is `base_footprint`, so `odom → base_footprint` was never bridged|`base_link_frame: base_footprint` in `ekf.yaml`|
|EKF appeared to hang when the robot was stationary|The driver only published `/odom` when serial data arrived; with zero motion, the topic went silent and starved the filter|`/odom` is now published on every 20 Hz timer tick regardless of whether new serial data arrived|
|EKF took up to 4.6 s to process a frame; robot appeared to slide sideways in RViz while stationary|EKF frequency was too high for the Pi's CPU, and fusing IMU linear acceleration let sensor noise integrate into phantom Y-axis drift|`frequency: 20.0`, `two_d_mode: true`; Y velocity is hardcoded to `0.0` in the driver; IMU linear acceleration is excluded from `imu0_config`|
|Driving forward produced zero net motion in RViz|Motors are mounted opposite each other — forward drive makes one encoder count negative and the other positive, so naive `(left + right) / 2.0` cancelled to ~0|One encoder channel's sign is inverted in software (`left_ticks = -int(parts[0])`)|
|`cmd_vel` values (m/s) sent straight to the ESP32 produced no motor movement|The firmware expects an 8-bit PWM value (0–255), not a velocity in m/s|Driver maps `linear_velocity * pwm_multiplier` and clamps to `[-255, 255]` before writing to serial|

### 6.2 ESP32 firmware (Arduino C++)

Reads `L:<pwm>,R:<pwm>\n` commands from the Pi over USB serial (115200 baud) and streams `ENC:<ticksA>,<ticksB>\n` telemetry back at 20 Hz.

```cpp
#include <Arduino.h>

// --- Pin Definitions (Adjust for your specific wiring) ---
const int PIN_PWM_A = 2;
const int PIN_AIN1 = 7;
const int PIN_AIN2 = 10;

const int PIN_PWM_B = 8;   // NOTE: conflicts with BNO085 SDA — see §7.2
const int PIN_BIN1 = 0;
const int PIN_BIN2 = 1;

const int PIN_ENC_A1 = 3;
const int PIN_ENC_A2 = 4;
const int PIN_ENC_B1 = 5;
const int PIN_ENC_B2 = 6;

volatile long encoder_a_ticks = 0;
volatile long encoder_b_ticks = 0;
int target_speed_a = 0;
int target_speed_b = 0;

void IRAM_ATTR handleEncoderA() {
  if (digitalRead(PIN_ENC_A2) == HIGH) encoder_a_ticks++;
  else encoder_a_ticks--;
}

void IRAM_ATTR handleEncoderB() {
  if (digitalRead(PIN_ENC_B2) == HIGH) encoder_b_ticks++;
  else encoder_b_ticks--;
}

void setMotorSpeeds(int speedA, int speedB) {
  // Motor A Control
  if (speedA >= 0) {
    analogWrite(PIN_PWM_A, speedA);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else {
    analogWrite(PIN_PWM_A, -speedA);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  }

  // Motor B Control
  if (speedB >= 0) {
    analogWrite(PIN_PWM_B, speedB);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
  } else {
    analogWrite(PIN_PWM_B, -speedB);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_PWM_A, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWM_B, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_ENC_A1, INPUT_PULLUP); pinMode(PIN_ENC_A2, INPUT_PULLUP);
  pinMode(PIN_ENC_B1, INPUT_PULLUP); pinMode(PIN_ENC_B2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A1), handleEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B1), handleEncoderB, RISING);
}

void loop() {
  // 1. Parse incoming PWM commands (Format: "L:127,R:127\n")
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (sscanf(input.c_str(), "L:%d,R:%d", &target_speed_a, &target_speed_b) == 2) {
      setMotorSpeeds(target_speed_a, target_speed_b);
    }
  }

  // 2. Stream encoder telemetry (Format: "ENC:ticksA,ticksB\n")
  Serial.print("ENC:");
  Serial.print(encoder_a_ticks);
  Serial.print(",");
  Serial.println(encoder_b_ticks);

  delay(50); // 20Hz telemetry loop
}
```

### 6.3 ROS 2 base driver (Python) — `base_driver.py`

Calculates kinematics, maps `/cmd_vel` to the `L:%d,R:%d` PWM protocol above, and publishes `/odom` with the covariance fields the EKF needs — continuously, at 20 Hz, even while stationary (see §6.1).

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import serial
import math

class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(0.05, self.update_odometry) # 20Hz

        # Physical Constants (calibrated — see §6.4)
        self.wheel_radius = 0.035
        self.track_width = 0.17
        self.ticks_per_rev = 43.2  # Calibrated CPR

        # Odometry State
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = self.get_clock().now()

        try:
            self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info("Connected to ESP32")
        except serial.SerialException:
            self.serial_conn = None
            self.get_logger().error("Failed to connect to ESP32")

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        left_speed = linear - (angular * self.track_width / 2.0)
        right_speed = linear + (angular * self.track_width / 2.0)

        # Convert m/s to 0-255 PWM
        pwm_multiplier = 255.0
        left_pwm = max(min(int(left_speed * pwm_multiplier), 255), -255)
        right_pwm = max(min(int(right_speed * pwm_multiplier), 255), -255)

        if self.serial_conn and self.serial_conn.is_open:
            cmd_str = f"L:{left_pwm},R:{right_pwm}\n"
            self.serial_conn.write(cmd_str.encode('utf-8'))

    def update_odometry(self):
        if not self.serial_conn or not self.serial_conn.is_open: return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        try:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line.startswith("ENC:"):
                    parts = line[4:].split(',')
                    left_ticks = -int(parts[0])  # Polarity inversion for left motor
                    right_ticks = int(parts[1])

                    if self.last_left_ticks is None:
                        self.last_left_ticks = left_ticks
                        self.last_right_ticks = right_ticks
                        self.last_time = current_time
                        return

                    d_left = 2.0 * math.pi * self.wheel_radius * ((left_ticks - self.last_left_ticks) / self.ticks_per_rev)
                    d_right = 2.0 * math.pi * self.wheel_radius * ((right_ticks - self.last_right_ticks) / self.ticks_per_rev)

                    self.last_left_ticks, self.last_right_ticks = left_ticks, right_ticks

                    d_center = (d_left + d_right) / 2.0
                    d_theta = (d_right - d_left) / self.track_width

                    self.x += d_center * math.cos(self.th + d_theta / 2.0)
                    self.y += d_center * math.sin(self.th + d_theta / 2.0)
                    self.th += d_theta

                    if dt > 0:
                        self.vx = d_center / dt
                        self.vth = d_theta / dt

                    self.last_time = current_time
        except Exception as e:
            pass

        # Publish Odometry — happens every timer tick, not just on new serial data
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0  # Anchor Y velocity to prevent drift
        odom.twist.twist.angular.z = self.vth

        # EKF Confidence Matrices
        odom.pose.covariance[0] = 0.01; odom.pose.covariance[7] = 0.01; odom.pose.covariance[35] = 0.01
        odom.twist.covariance[0] = 0.01; odom.twist.covariance[7] = 0.01; odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6.4 EKF configuration — `config/ekf.yaml`

Targets `base_footprint` (not `base_link` — see the TF tree lesson in §6.1), runs at 20 Hz to keep the Pi's CPU from falling behind, and excludes IMU linear acceleration from fusion to stop phantom sideways drift.

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 20.0
    two_d_mode: true
    publish_tf: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom

    odom0: /odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,  # vx, vy (vy is hardcoded 0 to prevent drift)
                   false, false, true,   # vth
                   false, false, false]
    odom0_differential: false

    imu0: /imu/data
    imu0_config: [false, false, false,
                   false, false, true,   # yaw
                   false, false, false,
                   false, false, true,   # vyaw
                   false, false, false]
    imu0_differential: false
```

### 6.5 Odometry distance calibration

If the physical distance driven doesn't match the distance shown in RViz, `ticks_per_rev` is wrong for your specific wheel/encoder pairing and needs a one-time calibration:

1. Drive the robot exactly 1.0 physical meter using teleop.
2. Note the X coordinate reported by `base_driver` (e.g. `0.030 m`).
3. Apply: `New CPR = Old CPR * (RViz distance / Real distance)` — e.g. `1440.0 * (0.030 / 1.0) = 43.2`.
4. Update `self.ticks_per_rev` in `base_driver.py` with the new value.

The `ticks_per_rev = 43.2` value already in §6.3 is the result of this calibration on the tested unit — re-run it if you change wheels, encoders, or gearing.

### 6.6 Teleoperation

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="<PI_IP_ADDRESS>:11811"   # use 127.0.0.1 if SSH'd into the Pi
export ROS_SUPER_CLIENT=TRUE

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 7. Hardware Reference

### 7.1 Compute

- **Main host:** Raspberry Pi 4, Ubuntu 22.04
- **Low-level base controller:** ESP32-C3, USB to Pi at `/dev/ttyACM0`, 115200 baud
- **Visualization/control station:** Laptop running Docker

### 7.2 ESP32-C3 Wiring — Base Controller

**TB6612FNG motor driver (drive motors)**

- Motor A (Left): PWM = GPIO 2, AIN1 = GPIO 7, AIN2 = GPIO 10
- Motor B (Right): PWM = GPIO 8, BIN1 = GPIO 0, BIN2 = GPIO 1

**N25 6V motor encoders**

- Encoder A: Phase A1 = GPIO 3 (interrupt, RISING), Phase A2 = GPIO 4
- Encoder B: Phase B1 = GPIO 5 (interrupt, RISING), Phase B2 = GPIO 6

**BNO085 IMU**

- VIN = 3.3V, GND = GND, SDA = GPIO 8, SCL = GPIO 9

> ⚠️ **Unresolved pin conflict, carried over verbatim from the source notes:** BNO085 SDA is assigned to GPIO 8, which is the same pin as Motor B's PWM. This was flagged but not fixed in the original documentation — re-map one of the two before wiring the final build. Likely candidates: move Motor B PWM to a free PWM-capable pin, or move IMU SDA/SCL to the ESP32-C3's alternate I2C pins, whichever leaves fewer other conflicts on your specific board revision.

### 7.3 Raspberry Pi Wiring — Sensors & Sweep

**TFmini-S LiDAR**

- Interface: hardware UART (`/dev/serial0`) at 115200 baud
- Config: requires `enable_uart=1` in `/boot/config.txt`, and Bluetooth disabled to free the serial port
- Power: requires a stable 5V rail — checksum failures are usually a power issue, not a wiring issue

**Sweep stepper motor (5V 28BYJ-48 + ULN2003 driver)**

- IN1 = Pi GPIO 17, IN2 = Pi GPIO 27, IN3 = Pi GPIO 22, IN4 = Pi GPIO 24
- Power: separate dedicated 5V supply — do not power from the Pi's own 5V rail (brownout risk)

**Intel RealSense D455 depth camera**

- Connected via USB 3.0

---

## 8. Software Stack & Launch Commands

**ESP32 firmware (Arduino C++)**

- Control loop at 20 Hz (50 ms delay)
- Command intake: serial strings from the Pi, format `L:<pwm>,R:<pwm>\n` (see §6.2 — an earlier protocol used a bare `SPEED_A,SPEED_B\n`; that version is superseded and not compatible with this firmware)
- Telemetry output: encoder tick counts back to the Pi, format `ENC:ticksA,ticksB\n`

**Building the workspace (Pi)** — whenever files are added or modified:

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

**Running the robot stack (Pi)** — using Method B (recommended):

```bash
sudo chmod 666 /dev/ttyACM0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="192.168.1.10:11811"
export ROS_SUPER_CLIENT=TRUE
ros2 launch robot_base_driver robot_bringup.launch.py
```

**Mapping/navigation (Pi)**

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/pi/ros2_ws/src/robot_base_driver/config/mapper_params_online_async.yaml
```

**Visualizing in RViz2 (laptop, inside Docker)**

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="192.168.1.10:11811"
export ROS_SUPER_CLIENT=TRUE
ros2 daemon stop
ros2 run rviz2 rviz2
```

---

## 9. Debugging Log — What Actually Broke, and Why

This section documents a real debugging session on this system, kept in narrative form because the _sequence_ of failures is informative.

### 9.1 Root causes (three interacting issues)

1. **Python node discovery-server isolation (`rclpy` behavior).** C++ nodes pick up `ROS_DISCOVERY_SERVER` without extra effort. Python nodes can silently drop off or fail to register with a remote server unless explicitly designated a **Super Client** via `ROS_SUPER_CLIENT=TRUE`. Without it, Python nodes quietly fall back to standard multicast/peer discovery and become invisible to the rest of the network — with no error message pointing at the cause.
2. **FastDDS XML schema strictness (Method A).** A custom XML profile that used generic tags like `<servers>` instead of the exact required structure (`<RemoteServer>`) caused `[XMLPARSER Error]` failures such as `Invalid element found into 'discoverySettingsType'`. FastDDS's XML parser is rigid; a malformed profile fails to load and the middleware silently falls back to defaults, which breaks communication with no obvious symptom besides "nothing works."
3. **Daemon and environment cache pollution.** The background `ros2 daemon` caches the node graph. Switching between local testing (`unset ROS_DISCOVERY_SERVER`) and networked/server configurations left the daemon holding a stale map, and lingering exported variables in old shells caused intermittent, hard-to-reproduce failures.

### 9.2 Remedies / best practices going forward

- **Always bundle the Super Client exports together** when running sensor drivers or Python nodes on an edge device tied to a central Discovery Server:
    
    ```bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DISCOVERY_SERVER="<SERVER_IP>:11811"
    export ROS_SUPER_CLIENT=TRUE
    ```
    
- **Avoid custom FastDDS XML unless you specifically need Method A.** Prefer the Discovery Server's plain environment variables. If XML is unavoidable, copy from a verified template rather than hand-rolling tags — the schema does not forgive near-misses.
    
- **Purge the daemon on every networking change**, before checking topics:
    
    ```bash
    ros2 daemon stop
    ```
    
- **Isolate terminal environments.** Keep terminals strictly dedicated to either server-hosting or node-execution roles, so exported variables from one role don't bleed into another and produce intermittent failures.
    

### 9.3 Quick verification sequence (Discovery Server method)

Run this in any fresh terminal after exporting the three variables from §4.2:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="192.168.1.10:11811"
export ROS_SUPER_CLIENT=TRUE
ros2 daemon stop
ros2 topic list
```

If `/scan`, `/imu/data`, `/odom`, and `/map` don't all show up, check in this order: (1) is the discovery server process actually running on the Pi, (2) is `ROS_SUPER_CLIENT` set in _this specific_ shell, (3) did you `ros2 daemon stop` after the last networking change, (4) is the corresponding node actually alive (`ros2 node list` from a shell on the same machine as the node, over SSH if needed).

---

## Appendix A — Part 7 Symptom Reference (Method A only)

|Symptom|Likely cause|Fix|
|---|---|---|
|`[XMLPARSER Error] Error opening '...'`|File not found — bind mount missing or container recreated|Re-check the `-v` mount, confirm `cat $FASTRTPS_DEFAULT_PROFILES_FILE` prints XML|
|`[XMLPARSER Error] Invalid element found into 'discoverySettingsType'`|Malformed XML tag (e.g. `<servers>` instead of `<RemoteServer>`)|Copy from a verified template; don't hand-write discovery-server XML|
|`ros2 topic list` shows topics via daemon, but `rviz2`/`echo` see nothing|Hardcoded single metatraffic port, or stale daemon|Don't hardcode a port in `metatrafficUnicastLocatorList`; `ros2 daemon stop`|
|Robot works locally, laptop sees nothing|WiFi AP is blocking/mangling multicast|Switch to unicast peers (Method A) or Discovery Server (Method B)|
|Python node invisible despite correct `ROS_DISCOVERY_SERVER`|Missing `ROS_SUPER_CLIENT=TRUE`|Export it in every shell running `rclpy` nodes|

## Related
- [[ROS/ROS odometry with sensor fusion]]
- [[ROS/ROS2 youtube]]
- [[ROS/1. Basics]]
- [[Toradex/UART Data Read]]
