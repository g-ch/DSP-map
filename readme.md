# Description
This repository contains the head files of the Dual-Structure Particle-based (DSP) map. This branch is to calculate risk maps, which are used to build risk-aware spatio-temporal (RAST) safety corridors for dynamic uncertain environments.

# Prerequest
Please install and test the [main branch](https://github.com/g-ch/DSP-map) of the DSP map first. If you can successfully run the demo in the main branch, you've got every dependency you need.


# Parameters
There are quite a few parameters in this map. We put the parameters in ``map_parameters.h`` in this branch. Only `Camera FOV` is coupled with hardware and should be modified according to the real FOV of your camera.
You can use default values for other parameters. The meanings of the parameters are as follows.

## Static parameters
The following parameters can be found at the top of the map head file.
1. Camera FOV. It is necessary to set the camera FOV angle for your camera. The unit is degree and we set the half-angle value.
    ```
    const int half_fov_h = 45;  // Half of the horizental angle. should be able to be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make half_fov_h a little smaller value than the real FOV angle
    const int half_fov_v = 27;  // Half of the vertical angle. Should be able to be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make half_fov_h a little smaller value than the real FOV angle
    ```
* The ``ANGLE_RESOLUTION`` is 3 in the head files. Don't change ``ANGLE_RESOLUTION`` unless you are very familiar with the way our map works.
* Tips: Depth camera usually contains large noise near the edge of the image. You can make the FOV parameter a little smaller than the real FOV angle. But never make it larger.

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



