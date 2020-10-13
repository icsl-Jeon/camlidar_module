# Camlidar module 

---
## Installation 

### Dependency 



#### Bluefox 

1. install bluefox driver

```bash
cd camlidar_module/camera_drivers/

unzip mvBlueCOUGAR_driver.zip -d mvBlueCOUGAR_driver

cd mvBlueCOUGAR_driver/matrixvision

sudo chmod +x install_mvGenTL_Acquire.sh

```

2. bluefox_ros

```bash
git clone https://github.com/icsl-Jeon/bluefox_ros.git

cd ~/catkin_ws

catkin build bluefox_ros
```

#### Velodyne (option)

```bash
https://github.com/ChanghyeonKim93/velodyne.git
```

#### px4_code (option)
```bash
https://github.com/icsl-Jeon/px4_code.git
```

#### vijeon (option)
```base
https://github.com/icsl-Jeon/vijeon.git
```

## HSV target localization 

### Description 
1. Input image for HSV thresholding 
We threshold the undistored image (member var of `CamLidarSyncAlign` class) not an original image obtained from bluefox camera. 

### Possible issues 