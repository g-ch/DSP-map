//
// Created by clarence on 2021/12/17.
//

#ifndef DYNAMIC_OCCPUANCY_MAP_MAP_PARAMETERS_H
#define DYNAMIC_OCCPUANCY_MAP_MAP_PARAMETERS_H

/** Parameters for the map **/
#define MAP_LENGTH_VOXEL_NUM 65 //33//29  //odd
#define MAP_WIDTH_VOXEL_NUM 65 //33//29   //odd
#define MAP_HEIGHT_VOXEL_NUM 35 //9 //21, 9 //odd
#define VOXEL_RESOLUTION 0.15
#define ANGLE_RESOLUTION 3
#define MAX_PARTICLE_NUM_VOXEL 18 // 18 //80
#define LIMIT_MOVEMENT_IN_XY_PLANE 1

/// Note: RISK_MAP_NUMBER * RISK_MAP_PREDICTION_TIME = PREDICTION_TIMES. RISK_MAP_PREDICTION_TIMES items in prediction_future_time should be within time a_star_search_time_step
#define PREDICTION_TIMES 9
#define RISK_MAP_NUMBER 3
#define RISK_MAP_PREDICTION_TIMES 3

static const float prediction_future_time[PREDICTION_TIMES] = {0.1f, 0.3f, 0.5f, 0.7f, 0.9f, 1.1f, 1.3f, 1.5f, 1.8f}; //unit: second
//static const float prediction_future_time[PREDICTION_TIMES] = {0.1f, 0.2f, 0.4f, 0.6f, 0.8f, 1.f, 1.2f, 1.4f, 1.6f, 1.8f, 2.f}; //unit: second

const int half_fov_h = 45;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make half_fov_h a smaller value than the real FOV angle
const int half_fov_v = 27;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make half_fov_h a smaller value than the real FOV angle

//const int half_fov_h = 30;  // Real world can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make half_fov_h a smaller value than the real FOV angle
//const int half_fov_v = 21;

#define CONSIDER_LOCALIZATION_UNCERTAINTY true


/** END **/

static const int VOXEL_NUM = MAP_LENGTH_VOXEL_NUM*MAP_WIDTH_VOXEL_NUM*MAP_HEIGHT_VOXEL_NUM;
static const int PYRAMID_NUM = 360*180/ANGLE_RESOLUTION/ANGLE_RESOLUTION;

#endif //DYNAMIC_OCCPUANCY_MAP_MAP_PARAMETERS_H
