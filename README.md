## volumetry-visionaryt

This repository contains a simple usage program made in C++ to estimate the volume of objects from a point cloud data acquired with the SICK® Visionary-T ToF Camera.

### Instalation

The recommended operational system to use this program is [Debian 9](https://www.debian.org/releases/stretch/), and the executable is generated with [CMake](https://cmake.org/). The libraries used are the integration C++ library to communicate with Visionary-T (provided along with the Visionary-T user manual and included in this repository inside the VisionaryCommon folder) and the [Point Cloud Library](https://pointclouds.org/), used to process the point cloud data.

First of all, it's necessary to install CMake and PCL

```
sudo apt-get install cmake
sudo apt-get install libpcl-dev

```

After that, turn on the Visionary-T ToF Camera and edit the main function of the 'volume-visionaryt.cpp' file, replacing the Camera IP

```
int main()
{
 runStreamingDemo((char*)"192.168.140.8", 2114);
}
```

Finally, you'll be able to generate the executable with the cmake command

```
cmake CMakeLists.txt
```

### Calibration

Before using the program, it's necessary to calibrate its filters to the camera placing set. The program uses 3 coordinate filters to cut off the points outside the interest region, and their limits are defined in the following lines of the 'volume-visionaryt.cpp' file

```
passx.setInputCloud (cloud);
passx.setFilterFieldName ("x");
passx.setFilterLimits (-0.23, 0.25);
passx.filter (*cloud_filtered);

passy.setInputCloud (cloud_filtered);
passy.setFilterFieldName ("y");
passy.setFilterLimits (-0.15, 0.26);
passy.filter (*cloud_filtered);

passz.setInputCloud (cloud_filtered);
passz.setFilterFieldName ("z");
passz.setFilterLimits (0, 0.76);
passz.filter (*cloud_filtered);
```

The X and Y coordinates are the area which the measured solids will be placed, so it's important to limit the X and Y filters only to the placement area to avoid the acquisition of points from surrounding objects. The Z coordinate is the distance of the Camera to the ground where the measured solids will be placed, so the Z filter needs to cut everything in a distance below the solid bottom coordinates (so it needs to cut off the ground!). 

The repository has a branch named 'calibration', that acquires a point cloud and plots it in the screen, so you can change the coordinate filters parameters and see what is being cut from the scene. I recommend to use a rectangular platform to put the measured objects above on it. The X and Y filters should cut the platform limits, and the Z filter should be put in the exact distance from the camera to the platform, such as it would return an empty point cloud when no object is placed on the platform. After finding the appropriate parameters you can copy them and update the 'volume-visionaryt.cpp' file in the 'master' branch, running cmake again to generate the executable file.

### Usage

After generating the executable file, the program can be executed

```
./volume-visionaryt
```
And the program will start to print the calculated volume in m³ and cm³. The program can be closed pressing any key
