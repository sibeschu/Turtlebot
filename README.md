# 1. Kamera kalibrieren

```bash
sudo apt install ros-jazzy-image-proc
sudo apt install ros-jazzy-image-pipeline # for calibrator

ros2 run camera_ros camera_node --ros-args -p width:=1152 -p height:=648 -p sensor_mode:="2304:1296"

ros2 run camera_calibration cameracalibrator --size 10x7 --square 0.026 --ros-args --remap image:=/camera/image_raw --remap camera:=/camera

rpicam-hello --list-cameras

# turtlebot@amrl-turtlebot1:~$ rpicam-hello --list-cam
# Available cameras
# -----------------
# 0 : imx708_wide_noir [4608x2592 10-bit RGGB] (/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a)
#     Modes: 'SRGGB10_CSI2P' : 1536x864 [120.13 fps - (768, 432)/3072x1728 crop]
#                              2304x1296 [56.03 fps - (0, 0)/4608x2592 crop]
#                              4608x2592 [14.35 fps - (0, 0)/4608x2592 crop]


```

# 2. Kamera starten

## auf Turtlebot 
```bash
ros2 run camera_ros camera_node --ros-args -p width:=1152 -p height:=648 -p sensor_mode:="2304:1296" -p camera_info_url:=file:///home/turtlebot/AutSys_Gruppe_1/ost.yaml -p orientation:=180
```

# 3. camera_raw -> camera_rect
## auf Host-PC
```bash
ros2 launch image_proc image_proc.launch.py namespace:=camera 
```
dann topic /image_rect_color

# 4. Birdseye / warpPerspective



### Dokumentation für camera_ros (camera_node parameter etc.)
https://github.com/christianrauch/camera_ros?tab=readme-ov-file#calibration

### Dokumentation für imx708
https://www.raspberrypi.com/documentation/accessories/camera.html#camera-module-3

# 2. Birdseye-View

TODO: /image_rect_color in Node birdseye_view.py abonnieren und mit openCV oder anderen Verfahren eine Verformung des Bildes so erzeugen, dass auf der Bildebene dem Horizont entgegenlaufenden Linien parallel werden

# 3. Pfadplanung

# 4. Netzwerk

sudo nano /etc/netplan/50-cloud-init.yaml
- IP für eth0 vergeben
auf PC Wired Verbindung mit IP im gleichen Adressraum festlegen
- Lankabel verbinden
