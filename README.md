
# 🛩️ Using the ArduPilot Simulation

![ArduPilot Logo](assets/ardupilotlogo.jpg)

For more information about ArduPilot, visit:  
👉 [https://ardupilot.org/](https://ardupilot.org/)

---

## 🖥️ Install Ubuntu 22.04.5

Download: [Ubuntu Jammy Release](https://releases.ubuntu.com/jammy/)

Update Ubuntu:
```bash
sudo apt update
sudo apt upgrade
```

---

## 🔧 Set Up the ArduPilot Build Environment

Reference: [ArduPilot Linux Build Setup](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)

Install Git:
```bash
sudo apt-get install git
```

Clone the ArduPilot repository:
```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

Install required packages:
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Reload the path:
```bash
. ~/.profile
```

Build the code:
```bash
./waf configure --board Pixhawk6c
./waf rover
```

---

## 🤖 Install ROS 2 (Humble)

Reference: [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Set Locale
```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

### Enable Universe Repository
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### Add ROS 2 APT Repository
```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### Update and Install ROS 2
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

### Test ROS 2 Nodes
Talker:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

Listener:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

---

## 🧰 ROS 2 Workspace Setup

Reference: [ArduPilot ROS 2 Integration](https://ardupilot.org/dev/docs/ros2.html)

Create workspace and import repos:
```bash
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
```

Initialize rosdep:
```bash
sudo rosdep init
rosdep update
```

Install dependencies:
```bash
cd ~/ardu_ws
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Install MicroXRCEDDSGen:
```bash
sudo apt install default-jre
cd ~/ardu_ws
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
source ~/.bashrc
microxrceddsgen -help
```

Build workspace:
```bash
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests
```

Run installation test:
```bash
cd ~/ardu_ws
source ./install/setup.bash
colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot --event-handlers=console_cohesion+
colcon test-result --all --verbose
```
