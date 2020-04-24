# Lidar Obstacle Detection

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />


**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

## Obstacle Detection
The main goal of the current project is to detect and cluster different potential obstacles in the scene. These objects can be other cars, cyclists, traffic signs, pedestrains ... etc. To achieve this goal the point cloud obtained from the LiDAR is processed to produce a number of 3D bounding boxes. Each bound box include a potential object.

The following steps was implementd:
1. The point cloud is downsampled to have a less resolution cloud. This cloud is computationaly cheaper for processing in real time. At the same time, the cloud resolution should not be too low in order not to skip any potential object.
2. The cloud has points from different objects that can be catogarized into 3 groups: potential obstacles, road, ego car roof  and side buildings. An ROI is determined to get the point located in the driving space only and neglected the points reflected from the buildings and the car roof. This can be done by using some information about car dimensions and road width. The later information can be obtained from a map or a camera.
3. The resulted cloud from the previous step contains only potential obstacles and road points. They are seperated by fitting the cloud points to a plane taking into account that the road is usually a plane (not always true). Then, the outliers which werenot fitted to the plane is the cloud points related to the obstacles. RANSAC algorithm was used to fit the points to the plane. Two implementation were proposed: PCL implementation and own implementation from scratch.
4. obstacle points then are clustered and bounded. Eculedian clustering using KDTree is used.Two implementation were proposed: PCL implementation and own implementation.
5. The final output is the number of bounding boxes with their associated 3D positions.

## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
> clone the repo
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
