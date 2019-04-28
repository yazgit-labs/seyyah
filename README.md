# SEYYAH- Mobile Crawler Robot Platform for Robotic Research

SEYYAH is a **ROS** based **mobile crawler robot** designed by **Ankara University Artificial Intelligence and Image Processing Labs (TR: YAZGİT labs)** to use in academical projects. It is a robot with a *360° laser range scanner*, a *camera*, *two wifi card*(to connect multiple networks at a time), *nvidia jetson tx2* power, *internal battery*. It also has an oled display that shows the current status and a led display that shows voltage status to make its developers job easier.

![Seyyah Front Image](img/seyyah.JPG)
![Seyyah Rear Image](img/seyyah2.JPG)

## Launch files

To drive the robot with a gamepad and get ready all parts of robot

```
roslaunch seyyah joy.launch
``` 

To drive the robot with keyboard and get ready all parts of robot

```
roslaunch seyyah keyboard_teleop.launch
``` 

To get ready all parts of robot

```
roslaunch seyyah bringup.launch
``` 

## Hardware parts

| Hardware Name                                     |Mission                                                    |Photo                                                                    |
|---------------------------------------------------|-----------------------------------------------------------|-------------------------------------------------------------------------|
|**Nvidia TX2 with Orbitty Carrier Board**          |`Main Computer`                                            |![Nvidia TX2 with Orbitty Carrier Board Image](img/orbitty_carrier.jpg)  |
|**Slamtec RPLIDAR A1**                             |`360° Laser Range Scanner`                                 |![Slamtec RPLIDAR A1 Image](img/rplidar_a1.jpg)                          |
|**Arduino MKR1000**                                |`Base Controller`                                          |![Arduino MKR1000 Image](img/arduino_mkr1000.jpg)                        |
|**Adafruit BNO055 Absolute Orientation Sensor**    |`IMU`                                                      |![Adafruit BNO055 Image](img/adafruit-bno055.jpg)                        |
|**Sparkfun TB6612FNG**                             |`Motor Driver`                                             |![Sparkfun TB6612FNG Image](img/tb6612fng.jpg)                           |
|**SZDoit Mini T100**                               |`Tracked Subframe`                                         |![SZDoit Mini T100 Image](img/szdoit_t100.jpg)                           |
|**USB Webcam**                                     |`Camera`                                                   |![USB Webcam Image](img/usb_webcam.jpg)                                  |
|**LM2596 5V 3A Step-Down Voltage Regulator**       |`Voltage Regulator For 5V Devices`                         |![LM2596 Image](img/LM2596.jpg)                                          |
|**3400 mAh 3S Lipo Battery**                       |`Power Supply`                                             |![Lipo Battery Image](img/lipo_battery.jpg)                              |
|**Voltmeter**                                      |`Battery Charge Status Display`                            |![Voltmeter Image](img/voltmeter.jpg)                                    |
|**I2C Oled Display**                               |`Robot Status Display(IP, Temp, Cpu, Mem...)`              |![Oled Display Image](img/i2c_oled.jpg)                                  |
|**TP-Link TL-WN725N**                              |`Second WiFi Network Adapter To Connect Multiple Networks` |![TL-WN725N Image Image](img/tlwn725n.jpg)                               |
|**Trust Vecco HU-4440P**                           |`USB HUB For Connecting Multiple USB Devices To Jetson`    |![Trust Vecco HU-4440P Image](img/usb_hub.jpg)                           |

#

> **Note:** Details and more will be added.

