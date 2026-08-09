# INSTALL

If you want to setup the project by yourself, here is a list to the common commands to execute.

## Install Ubuntu dual boot
Good video : [Install Ubuntu 24.04 LTS in Dual Boot With Windows 10/11 | Dual Boot Windows 11 and Ubuntu](https://www.youtube.com/watch?v=eaPVou9lXeU&t=13s)
## Setup Ubuntu server
<details> <summary>Click to expand</summary>

### Add a WIFI network
Follow the tuto : [Ubuntu Server: Connect to Wi-Fi from command line - LinuxConfig](https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line)
For raspi5 under Ubuntu 24.04:
```bash
ip a # Check if wifi/eth
ls /sys/class/net # Check nammming
sudo nano /etc/netplan/50-cloud-init.yaml
```
And copy
```bash
network:
	version: 2
	ethernets:
		eth0:
			dhcp4: true
			optional: true
		wifis:
			wlan0:
				optional: true
				dhcp4: true
				access-points:
					"GADZ_DUAV_222_24GHz": 
						password: "duav2225"   
```
Then do 
```bash
sudo netplan apply
# And check it is ok
ip a
```
### Pass keyboard to AZERTY
```bash
sudo dpkg-reconfigure keyboard-configuration # Choose config
sudo setupcon
```
</details>

## Setup GIT
<details> <summary>Click to expand</summary>

### Add name and email
```bash
git config --global user.name "..."
git config --global user.email "..."
```

### GIT Large File System (LFS)
Used for file >= 100MGb
To install: [Git LFS Install | GeeksforGeeks](https://www.geeksforgeeks.org/git-lfs-install/)
```bash
sudo apt-get install git-lfs
git lfs install
# To track a file
git lfs track "*.png"
git add .gitattributes
git commit -m "Track XXXX files with Git LFS"
# To view
git lfs ls-files
```

</details>

## SSH
If you need to delete ssh key on windows:
```bash
ssh-keygen -R 192.168.8.165    # In powershell
```
## ROS2 Humble
Middleware used to defined drone actions, services and communications topics (mi-high, mi-low level). It is also used for easy simulation. To install https://docs.ros.org/en/humble/index.html#.
Needs Ubuntu 22.04

## FSM python
Finite State Machine to control the drone at a high level
```bash
pip3 install python-statemachine 
pip3 install python-statemachine[diagrams]
# On 24.04
pip3 install python-statemachine --break-system-packages
pip3 install python-statemachine[diagrams] --break-system-packages
```
## Ardupilot
Used to control and simulate the drone a low-level. To install : https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux.
Puis exctuez dans le fichier ardupilot qui vient d'appraitre :
```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile # Then logout to apply changes
# To make the call to SITL from anywhere
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```
If you encounter an error with numpy, try:
```bash
pip install "numpy<1.28.0,>=1.21.6" scipy
pip install "numpy<1.28.0,>=1.21.6" scipy
```
## Pymavlink
Used to communicate with the a drone (or simulated drone - SITL) with mavlink commands at LOW level : [ArduPilot/pymavlink: python MAVLink interface and utilities](https://github.com/ArduPilot/pymavlink)
```bash
sudo apt-get install libxml2-dev libxslt-dev
sudo apt-get install python3-numpy python3-pytest
# Hors venv 22.04
sudo python -m pip install --upgrade future lxml
sudo python -m pip install --upgrade pymavlink
# With venv 24.04
pip install future lxml numpy pytest --break-system-packages
pip install pymavlink --break-system-packages
```
## Geopy
Used for precises calculations using GPS coordinates.
```bash
pip install geopy
pip install geopy --break-system-packages
```
## OpenCV
To install openCV:
```bash
pip install opencv-python
pip install opencv-python --break-system-packages
pip install opencv-python-headless
pip install opencv-python-headless --break-system-packages
```
OpenCV for image analysis and a it's bridge for ros2:
```bash
pip3 install cv_bridge 
pip3 install cv_bridge --break-system-packages
 ```
## rsync
For fast synchronization of builds :
[Rsync Command in Linux with Examples | Linuxize](https://linuxize.com/post/how-to-use-rsync-for-local-and-remote-data-transfer-and-synchronization/)
```bash
sudo apt install rsync
```

