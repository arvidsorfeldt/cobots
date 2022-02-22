# cobots
This must be installed on a system with ubuntu 20.04 focal with ros installed and sourced.

First install rust.
```console
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```
Then install dependencies.
```console
sudo apt install libclang-dev ros-galactic-xacro
```

Make install script executable
```console
chmod +x install.bash
```
Restart vscode and run the installer.
```console
./install.bash
```
To get all 3d models download stl files from drive and place them in /ros/ia_ros_meshes/
