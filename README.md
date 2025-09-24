# Using the ArduPilot Simulation

![ArduPilot Logo](assets/ardupilotlogo.jpg)

For more information about ArduPilot, visit the [official website](https://ardupilot.org/).

---

## 1. Install Ubuntu 22.04.5

You can find the installation files for Ubuntu 22.04.5 [here](https://releases.ubuntu.com/jammy/).

After installation, update Ubuntu with the following commands:
sudo apt update
sudo apt upgrade


---

## 2. Set Up the ArduPilot Build Environment

For detailed instructions, refer to the [official documentation](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux).

### Install Git
sudo apt-get install git


### Clone the ArduPilot Repository
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot


### Install Required Packages
Run the following command from the cloned repository:
Tools/environment_install/install-prereqs-ubuntu.sh -y


### Reload the Path
. ~/.profile


### Build the Code
./waf configure --board Pixhawk6c
./waf rover


---

## 3. Install ROS 2

For full installation instructions, see the [ROS 2 website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

### Setup Instructions

First, set your locale:
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings


Next, ensure the Ubuntu Universe repository is enabled:
sudo apt install software-properties-common
sudo add-apt-repository universe


Add the ROS 2 apt repository:
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s [https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest](https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest) | grep -F "tag\_name" | awk -F" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "[https://github.com/ros-infrastructure/ros-apt-source/releases/download/$](https://github.com/ros-infrastructure/ros-apt-source/releases/download/$){ROS\_APT\_SOURCE\_VERSION}/ros2-apt-source\_${ROS\_APT\_SOURCE\_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb


Update your apt repository caches:
sudo apt update
sudo apt upgrade


Install the desktop and development tools:
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools


### ROS 2 Test Code

To test the installation, run the talker and listener nodes in separate terminals.

**Talker Node**
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker


**Listener Node**
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener


---

## 4. ROS 2 Workspace Setup

Detailed instructions can be found in the [ArduPilot ROS 2 documentation](https://ardupilot.org/dev/docs/ros2.html).

### Clone Repositories
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src


### Initialize and Update Dependencies
sudo rosdep init
rosdep update
cd ~/ardu_ws
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y


### Install MicroXRCEDDSGen
sudo apt install default-jre
cd ~/ardu_ws
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=$PATH:$PWD/scripts" >> ~/.bashrc


Test the installation:
source ~/.bashrc
microxrceddsgen -help


### Build the Workspace
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests


### ArduPilot ROS 2 Installation Test
cd ~/ardu_ws
source ./install/setup.bash
colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot --event-handlers=console_cohesion+
colcon test-result --all --verbose


---

## 5. ROS 2 with SITL (Software in the Loop)

Learn more in the [ROS 2 SITL documentation](https://ardupilot.org/dev/docs/ros2-sitl.html).

### Run the ROS 2 SITL Simulator
source /opt/ros/humble/setup.bash
cd ~/ardu_ws/
colcon build --packages-up-to ardupilot_sitl
source install/setup.bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=(ros2pkgprefixardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501


Once running, you can interact with ArduPilot using the ROS 2 command-line interface (CLI).
source ~/ardu_ws/install/setup.bash

See the node appear in the ROS graph
ros2 node list

See which topics are exposed by the node
ros2 node info /ap

Echo a topic published from ArduPilot
ros2 topic echo /ap/geopose/filtered


You can also launch MavProxy in another terminal:
mavproxy.py --console --map --aircraft test --master=:14550


---

## 6. ROS 2 with Gazebo

Refer to the [ArduPilot Gazebo documentation](https://ardupilot.org/dev/docs/ros2-gazebo.html) for more details.

### Install Gazebo
Follow the installation steps on the [Gazebo website](https://gazebosim.org/docs/harmonic/install_ubuntu/).
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic


Install `ros_gz`:
sudo apt-get install ros-humble-ros-gzharmonic


Check the installation:
gz sim


### Install ArduPilot Gazebo
cd ~/ardu_ws
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src


Set the Gazebo version:
export GZ_VERSION=harmonic


Add Gazebo APT sources:
sudo apt install wget
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt upgrade


Add Gazebo sources to `rosdep`:
sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
rosdep update


Build the workspace:
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_gz_bringup


Test the installation:
cd ~/ardu_ws
source install/setup.bash
colcon test --packages-select ardupilot_sitl ardupilot_dds_tests ardupilot_gazebo ardupilot_gz_applications ardupilot_gz_description ardupilot_gz_gazebo ardupilot_gz_bringup
colcon test-result --all --verbose


### Run the Simulation
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py


Other available models:
* **iris Runway (Copter):** `ros2 launch ardupilot_gz_bringup iris_runway.launch.py`
* **Iris Maze (Copter):** `ros2 launch ardupilot_gz_bringup iris_maze.launch.py`
* **WildThumper (Rover):** `ros2 launch ardupilot_gz_bringup wildthumper.launch.py`

---

## 7. Cartographer SLAM with ROS 2 in SITL

For more information, visit the [ArduPilot documentation](https://ardupilot.org/dev/docs/ros2-cartographer-slam.html).

### Install
Clone the repository:
cd ~/ardu_ws/src
git clone https://github.com/ArduPilot/ardupilot_ros.git


Update `rosdep`:
cd ~/ardu_ws
rosdep install --from-paths src --ignore-src -r --skip-keys gazebo-ros-pkgs


Build:
cd ~/ardu_ws
source ./install/setup.bash
colcon build --packages-up-to ardupilot_ros ardupilot_gz_bringup


### Test
Run the simulation in one terminal:
source ~/ardu_ws/install/setup.sh
ros2 launch ardupilot_gz_bringup iris_maze.launch.py


In a separate terminal, launch Cartographer:
source ~/ardu_ws/install/setup.sh
ros2 launch ardupilot_ros cartographer.launch.py
