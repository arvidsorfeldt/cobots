# cobots
Start by making sure that you have ros2 galactic installed on ubuntu 20.04. Otherwise, follow this [guide](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
Make sure that the setup file is sourced with the command
```console
source /opt/ros/galactic/setup.bash
```
First install rust
```console
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```
Then install dependencies
```console
sudo apt install libclang-dev ros-galactic-xacro python3-colcon-common-extensions
```
Make install script executable
```console
chmod +x install.bash
```
**Restart vscode** and run the installer
```console
./install.bash
```
To get all 3d models download stl files from drive and place them in /ros/ia_ros_meshes/
