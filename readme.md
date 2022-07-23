# Description
This repository contains the head files of the Dual-Structure Particle-based (DSP) map
and a ROS node example to use this map. For more information about the DSP map, see [video](https://www.youtube.com/watch?v=seF_Oy4YbXo&t=5s) and [preprint](https://arxiv.org/abs/2202.06273).

There are one __head files__ in the `include` folder.
1. ``dsp_dynamic_omindirectional.h`` is the head file for the DSP map with a constant velocity model. This head file supports omnidirectional sensor like lidar.

Just include one of the head files in your source file and use the map. We write everything in a single head!

A ROS __node example__ `map_sim_example.cpp` can be found in the `src` folder.


# Compile
__Tested environment__: Ubuntu 18.04 + ROS Melodic and Ubuntu 20.04 + ROS Noetic

To compile the source code of our map, you need:
1. PCL and Mavros. PCL is included in the desktop-full version of ROS.
   Mavros is only used for ROS message subscriptions in the example node. Check [mavros](https://github.com/mavlink/mavros) for installation guidance.

2. Install [munkers-cpp](https://github.com/saebyn/munkres-cpp) with the following steps.
    ```
    git clone https://github.com/saebyn/munkres-cpp.git
    cd munkres-cpp
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    ```

3. Download and compile the example node
    ```
    mkdir -p map_ws/src
    cd map_ws/src
    git clone https://github.com/g-ch/DSP-map.git
    cd ..
    catkin_make
    ```

# Parameters
There are quite a few parameters in this map. But only `FOV` is coupled with hardware and should be modified according to the real FOV of your lidar.
You can use default values for other parameters.

## Static parameters
The following parameters can be found at the top of the map head file.
1. Sensor FOV. It is necessary to set the FOV angle for your lidar. The unit is degree and we set the half-angle value.
    ```
    const int HALF_FOV_H = 45;  // Half of the horizental angle. should be able to be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make HALF_FOV_H a little smaller value than the real FOV angle
    const int HALF_FOV_V = 27;  // Half of the vertical angle. Should be able to be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make HALF_FOV_H a little smaller value than the real FOV angle
    ```
* The ``ANGLE_RESOLUTION`` is 3 in the head files. Don't change ``ANGLE_RESOLUTION`` unless you are very familiar with the way our map works.

2. Change the size and resolution of the map by changing the following parameters:
    ```
    #define MAP_LENGTH_VOXEL_NUM 66
    #define MAP_WIDTH_VOXEL_NUM 66
    #define MAP_HEIGHT_VOXEL_NUM 40
    #define VOXEL_RESOLUTION 0.15
    ```
* The default resolution is `0.15m`. The real size of the Map length is `MAP_LENGTH_VOXEL_NUM * VOXEL_RESOLUTION`.
* With different resolutions, you also need to change the threshold in Function "getOccupancyMapWithFutureStatus" in the ROS node.

  __Note:__ Although the DSP map is continuous and composed of particles. We usually acquire the current or future occupancy status from the voxel structure. The voxel resolution is theoretically arbitrary but you need to tune the parameter for threshold and maximum particle number if the resolution is changed.
  To maintain the same efficiency when the resolution changes (the real map size is the same), the maximum particle number in the map shouldn't change, which means you need to change the maximum number of particles in each voxel space by changing the Parameter `MAX_PARTICLE_NUM_VOXEL`.


3. Change the time stamp to predict the future status
    ```
    #define PREDICTION_TIMES 6
    static const float prediction_future_time[PREDICTION_TIMES] = {0.05f, 0.2f, 0.5f, 1.f, 1.5f, 2.f}; //unit: second
    ```

## Dynamic parameters
The following parameters can be changed dynamically in the node.
```
my_map.setPredictionVariance(0.05, 0.05); // StdDev for prediction. velocity StdDev, position StdDev, respectively.
my_map.setObservationStdDev(0.1); // StdDev for update. position StdDev.
my_map.setNewBornParticleNumberofEachPoint(20); // Number of new particles generated from one measurement point.
my_map.setNewBornParticleWeight(0.0001); // Initial weight of particles.
DSPMap::setOriginalVoxelFilterResolution(res); // Resolution of the voxel filter used for point cloud pre-process.
```

# Record Particles
In our ROS node, we publish point cloud from occupancy status. The particles are not published because they are too many.
But you can record the particles at one time to a CSV file for analysis.

```
my_map.setParticleRecordFlag(1, 19.2); // Uncomment this to save particle file of a moment. Saving will take a long time. Don't use it in real-time applications.
```

You can use the Matlab app tool ``app1.mlapp`` in the `display` folder to open the CSV and view and analyze the particles.

# Citation
If you use our code in your research, please cite
```
@article{chen2022continuous,
  title={Continuous Occupancy Mapping in Dynamic Environments Using Particles},
  author={Chen, Gang and Dong, Wei and Peng, Peng and Alonso-Mora, Javier and Zhu, Xiangyang},
  journal={arXiv preprint arXiv:2202.06273},
  year={2022}
}
```

# License
MIT license.


