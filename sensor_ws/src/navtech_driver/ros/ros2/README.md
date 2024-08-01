# Navtech ROS2 Driver

The Navtech ROS2 Driver repository contains examples of ROS2 publishers and subscribers, which interface with RTSP cameras and Navtech Radar.
Both basic and complete examples are provided which allow simple publishing of data, simple subscribing to data, and also immediate viewing
of radar/camera data, using the ROS2 visulaisation tool, RVIZ.

Note - The Navtech ROS2 driver is dependent on the Navtech SDK.

Please see lower level README.md files, for more specific information on the ROS2 project folders.

## Navtech SDK Requirements

### C++ 17

* C++17 Compiler
Install with the following commands:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test

sudo apt-get update

sudo apt install gcc-9 gcc-9-base gcc-9-doc g++-9

sudo apt install libstdc++-9-dev libstdc++-9-doc

sudo apt install libbotan-2-dev
```

* GCC 9.x and above
Install with the following commands:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test

sudo apt update

sudo apt install gcc-9
```

* Clang 10 and above
Install with the following command:

```bash
sudo apt-get install clang-10
```

### Microsoft .NET
* .NET 4.8 and above

## Linux Requirements
To use the shell scripts provided in the 'utility scripts' folder we require bash on Ubuntu. First you must execute:
```bash
sudo dpkg-reconfigure -p critical dash
```

## ROS2 Requirements
As above and:
ROS2 Humble Hawksbill Geoclone - CPP and Python bindings
ROS2 Humble installations are accessible here: https://docs.ros.org/en/humble/Installation.html


## Python Requirements
Python3.0+

```bash
sudo apt install python3.8
```

Numpy 1.17.4
Can be installed with the following command:

```bash
sudo apt install python3-numpy
```

## OpenCV Requirements
OpenCV 4.5.3 - CPP and Python bindings
Installation instructions here: https://www.linuxfromscratch.org/blfs/view/svn/general/opencv.html
Note - this link has been updated, subsitute version number with 4.5.3

## FFMPEG Requirements
ffmpeg version 4.2.4-1ubuntu0.1
Can be installed with the following command:

```bash
sudo apt install ffmpeg
```

## License

The ROS2 driver which is released under The MIT License (MIT).
See file LICENSE.txt or go to <https://opensource.org/licenses/MIT> for full license details.

## ROS2 Packages

To use ROS commands, ROS2 must first be sourced using: source /opt/ros/humble/setup.bash
Note - this must be done in every new terminal
Alternatively, make this permanent by adding to your bash file: echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
This will source ROS for every newly opened terminal window

All ROS2 packages must be built and installed before being run
Build with:
```bash
colcon build
```

Install with:
```bash
. install/setup.bash
```

Packages can be run like so:

```bash
ros2 run <package_name> <executable_name>
```

For example:

```bash
ros2 run nav_ros colossus_publisher
```

Packages can be run with their corresponding paramater files like so:

```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <params_file_path>
```

For example:

```bash
ros2 run nav_ros colossus_publisher --ros-args --params-file ./src/nav_ros/config/colossus_publisher.yaml
```

## camera_ros

This folder contains examples based on a connection to an RTSP camera stream. Examples include conencting to and reading
camera images, publishing camera images, subscribing to camera images, and saving camera iamges to a video file. Sample
config file(s) are also included in the 'config' directory.

**See the README.md under 'camera_ros', for more detailed instructions**

## launch_ros

This folder contains launch files which are intended to receive and display data with ease. The launch files will launch the
required ros nodes, transform(s) and UI (RVIZ) to display data reived from a RTSP camera or Navtech Radar. The
Launch files also use configuration files which contain the default settings for the examples.

**See the README.md under 'launch_ros', for more detailed instructions**

## messages

Contains the custom message types used within the ROS2 Navtech driver - this includes camera messages and Navtech radar messages.

**See the README.md under 'messages', for more detailed instructions**

## nav_ros

This folder contains examples based on a connection to a Navtech radar. Examples include conencting to and reading
radar data, publishing radar data and images, subscribing to radar data and images, and saving radar iamges to a video file. Sample
config file(s) are also included in the 'config' directory.

**See the README.md under 'nav_ros', for more detailed instructions**

## rviz_views

Contains rviz view configuration files used to quickly visualise data output from the above packages. These view configuration
files are used by the launch files.

**See the README.md under 'rviz_views', for more detailed instructions**



# ROS2 Example install procedure

## Update Ubuntu 22.04

```bash
sudo apt update

sudo apt upgrade
```
	
## Install IASDK prerequisites

```bash
sudo apt install build-essential clang g++ protobuf-compiler libprotobuf-dev cmake
```
	
## Install ROS2 prerequisites

```bash
sudo apt install -y build-essential libssl-dev libffi-dev python3-dev python3 python3-pip software-properties-common
```

## Install ROS2

```bash
sudo apt update && sudo apt install curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade

sudo apt install ros-humble-desktop

source /opt/ros/humble/setup.bash

ros2 run demo_nodes_cpp talker
```
	
## Check install by running examples

```bash
source /opt/ros/humble/setup.bash

ros2 run demo_nodes_py listener
```

Check that the talker and listener are connected, and messages are being exchanged

## Install OpenCV

```bash
sudo apt install git

cd ~

git clone --depth 20 https://github.com/opencv/opencv.git

git clone --depth 20 https://github.com/opencv/opencv_contrib.git

sudo apt install build-essential cmake git pkg-config libpng-dev libtiff-dev gfortran openexr libgtk-3-dev libavcodec-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libatlas-base-dev python3-dev python3-numpy libtbb2 libtbb-dev libdc1394-dev libopenexr-dev

cd opencv

mkdir build

cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE    -D CMAKE_INSTALL_PREFIX=/usr/local          -D WITH_CUDA=OFF        -D INSTALL_PYTHON_EXAMPLES=ON          -D OPENCV_GENERATE_PKGCONFIG=ON         -D  OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules  -D OPENCV_ENABLE_NONFREE=ON         -D BUILD_EXAMPLES=ON ..

make -j8  // (replace the 8 by the number of usable cores on your machine, which is reported by the command nproc)

sudo make install

sudo ldconfig
```

## Check OpenCV is installed

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

check that version >=4.5.3 is reported
	
```bash
pkg-config --modversion opencv4
```
	
check that version >=4.5.3 is reported
	
## Install the Navtech Radar IASDK

```bash
git clone https://bitbucket.org/navtechradar/iasdk-public.git
```

## Install Colcon (the ROS2 build tool)

sudo apt install libbotan2-dev

sudo apt install python3-colcon-common-extensions

cd ~/iasdk-public/cpp/cpp_17

colcon build
```

Check the above command does not produce any errors

##Patch some files (temporary step)
Copy the contents of the provided patch folder "nav_radar" to replace ~/iasdk-public/ros/ros2/src/nav_radar


## Build the ROS2 IASDK packages

```bash
cd ~/iasdk-public/ros/ros2/

source /opt/ros/humble/setup.bash

colcon build

. install/setup.bash
```

## Common errors and solutions

# Ros executables cannot be found after build and execution attempt

Run the following to source the newly build executables

```bash
source /opt/ros/humble/setup.bash

. install/setup.bash
```

# Unable to launch RVIZ due to missing library (libQt5Core.so.5)

Run the following to source the newly build executables

```bash
rviz2: error while loading shared libraries: libQt5Core.so.5: cannot open shared object file: No such file or directory
```

In short - the above command removes a piece of code which is looking for something (renameat2 system call) that does not exist in kernels < 3.15

# Debian package build instructions for Navtech ROS2 SDK

## Install the protobuf compiler

```bash
sudo apt install protobuf-compiler
```

## Install the build tools

```bash
    sudo apt install python3-bloom python3-rosdep fakeroot debhelper dh-python

    rosdep init

    rosdep update
```

## Build and install the ROS packages

```bash
cd iasdk/ros/ros2

colcon build

. install/setup.bash
```

## Create YAML to point towards custom ROS packages

Create a yaml file (in the workspace containing the packages, for example) specifying the local packages. Example of yaml content:
(Add an entry for all required packages)

navtech_msgs:
    ubuntu: [nav-messages]

## Create a file in the rosdep sources folder

```bash
sudo nano /etc/ros/rosdep/sources.list.d/10-local.list
```

Add to this file the absolute path of the .yaml file created earlier. Example:

yaml file:///absolute_path_to_local_rosdep_file_list.yaml

## Update the ros dependencies

```bash
rosdep update
```

## Check if the new rosdep packages lists are being correctly found using:

```bash
rosdep resolve package_name
```

## Now build the debian packages

```bash
cd ~/iasdk-public/ros/ros2/src/navtech_msgs

bloom-generate rosdebian --os-name ubuntu --ros-distro humble

cd ~/iasdk-public/ros/ros2/src/navtech_msgs/debian
```

Edit the file "control" and change package name from ros-humble-nav-messages to nav-messages

```bash
run fakeroot debian/rules binary
```

This should build a .deb package called navtech_msgs, in iasdk-public/ros/ros2/src

cd to the next ros2 package root and repeat the above steps

# Install debian packages and check ROS can find them

Note - navtech_msgs package must be installed first, and nav_launch package must be installed after nav_camera and nav_radar.
This is due to packages being dependent on other pacakges

```bash
sudo dpkg -i nav-messages_1.0.0-0jammy_amd64.deb
```

Repeat the above for all Navtech ROS packages that you require

## Check ROS can find an installed package

```bash
 ros2 pkg prefix navtech_msgs
```

The above should show the location of the installed package. Example: /opt/ros/humble