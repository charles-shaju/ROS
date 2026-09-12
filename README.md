# Robot SLAM Setup & Debugging Guide

**Raspberry Pi 4 + ROS 2 Humble + slam_toolbox + RViz on a laptop over WiFi (FastDDS unicast)**

Version 1.0 — September 2026. Tested end-to-end on the setup described below. Follow the parts in order; the debugging section (Part 7) is written so you can jump straight to a symptom.

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

| Item | Value |
|---|---|
| Robot computer | Raspberry Pi 4, Ubuntu 22.04 (64-bit), hostname `raspberrypi`, IP 192.168.1.10 |
| Visualization computer | Laptop running a ROS 2 Docker container, IP 192.168.1.3 |
| ROS 2 distribution | Humble Hawksbill (Python 3.10) |
| DDS middleware | FastDDS 2.6.x (`rmw_fastrtps_cpp`, the Humble default) |
| SLAM | slam_toolbox, `online_async` mode |
| Laser | TFmini on a rotating sweep mount, publishing `sensor_msgs/LaserScan` on `/scan` (real range 0.1-12 m) |
| IMU | BNO085 via I2C (`/imu/data`) |
| Base | ESP32 motor driver interface via USB (`/dev/ttyACM0`), subscribes `/cmd_vel` |
| Odometry fusion | `robot_localization` EKF: `/odom` + `/imu/data` -> `/odometry/filtered` |
| URDF frames | base_footprint, base_link, laser_frame, imu_link, camera_link |

### Three golden rules (read these first)

1. Every ROS 2 process is its own DDS participant. Every participant must be started in a shell where the FastDDS environment variables are set. Exports never "carry over" to new terminals.
2. The `ros2` command-line tools (topic list, echo, rviz2) talk to a background daemon that caches the graph. A stale daemon lies to you. After changing any DDS/network setting: `ros2 daemon stop && ros2 daemon start`.
3. A Docker container's filesystem is temporary. Any file you create inside it (XML profiles, RViz configs) is deleted when the container is recreated. Mount files in from the host.

---

## Part 1 — Installation

### 1.1 Raspberry Pi (robot)

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

Check the slam params file `~/ros2_ws/src/robot_base_driver/config/mapper_params_online_async.yaml`: the laser range limits must match the TFmini, otherwise slam_toolbox prints `minimum/maximum laser range setting exceeds the capabilities of the used Lidar`:

```yaml
min_laser_range: 0.1    # NOT 0.0
max_laser_range: 12.0   # NOT 25.0
```

### 1.2 Laptop (Docker)

Use host networking (ROS 2 + Docker without `--net=host` is a world of pain), mount the DDS profile from the host, and pass the GUI through:

```bash
xhost +local:docker    # once per laptop boot, allows the container to open windows

mkdir -p ~/ros         # host-side home for files that must survive the container
# -> put fastdds_unicast.xml (section 2.3, laptop version) into ~/ros first

docker run -it --net=host --ipc=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/ros/fastdds_unicast.xml:/root/fastdds_unicast.xml \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_unicast.xml \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  osrf/ros:humble-desktop
```

Inside the container, make the exports permanent for shells opened later with `docker exec`:

```bash
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_unicast.xml' >> /root/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /root/.bashrc
source /opt/ros/humble/setup.bash
```

If you skip the bind mount, the XML file evaporates the next time the container is recreated and you get the classic symptom: `[XMLPARSER Error] Error opening '/root/fastdds_unicast.xml'`. FastDDS prints that when the file named by `FASTRTPS_DEFAULT_PROFILES_FILE` cannot be opened, then silently continues with default (multicast) settings — nothing works and the only clue is that one line.

---

## Part 2 — Network and FastDDS unicast (the hard part)

### 2.1 Why this is needed

ROS 2 discovers nodes using UDP multicast. Many WiFi routers/access points block or mangle multicast (AP isolation), which produces exactly this pattern: the robot works perfectly locally, but the laptop sees nothing, some topics, or stale topics. The fix is to disable reliance on multicast and tell every participant exactly where its peers are: a FastDDS "initial peers" list. When an initial peers list is defined, FastDDS sends discovery announcements only to those addresses — so the list must include both the other machine and the local machine itself (loopback), otherwise nodes on the same computer stop seeing each other. (Shared-memory transport does not reliably rescue same-host discovery when initialPeersList is used — see rmw_fastrtps issue #676.)

### 2.2 The port math (so the XML makes sense)

Each participant listens for discovery ("metatraffic") on its own well-known UDP port:

`port = 7400 + 250 * domainID + 10 + 2 * participantID`

With the default domain 0:

| Participant ID on a machine | Metatraffic port |
|---|---|
| 0 | 7410 |
| 1 | 7412 |
| 2 | 7414 |
| 3 | 7416 |
| 4 | 7418 |
| 5 | 7420 |
| 6 | 7422 |
| 7 | 7424 |

You cannot know in advance which ID each node gets (it depends on startup order, and the ros2 daemon takes one too), so the peers list simply enumerates IDs 0-7 for each address. Extra entries are harmless.

Do NOT hardcode `<port>7410</port>` in `metatrafficUnicastLocatorList`: that forces every participant on the machine to fight over one port, and only one of them wins (typical symptom: `ros2 topic list` shows everything via the daemon, but rviz2/echo see nothing).

### 2.3 The XML profiles

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

Laptop version — save as `~/ros/fastdds_unicast.xml` on the host (it is mounted to `/root/fastdds_unicast.xml` in the container). Identical except: the second metatraffic address is `192.168.1.3`, and the remote block (the last eight entries) uses `192.168.1.10`. The loopback block stays the same — rviz2, the daemon and CLI tools inside the container need it to see each other.

### 2.4 Environment variables (every terminal, both machines)

```bash
# Pi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_unicast.xml

# Laptop (inside Docker)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_unicast.xml
```

Persist them in `~/.bashrc` (Pi) and `/root/.bashrc` (container). `FASTRTPS_DEFAULT_PROFILES_FILE` is the correct variable name for Humble's FastDDS 2.6.x.

Sanity check before launching anything, in every new shell:

```bash
echo $FASTRTPS_DEFAULT_PROFILES_FILE   # must print the path
cat $FASTRTPS_DEFAULT_PROFILES_FILE    # must print the XML (NOT "No such file")
```
