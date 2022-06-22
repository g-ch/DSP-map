//
// Created by clarence on 2021/8/19.
//

#include <ctime>
#include <cmath>
#include <fstream>
#include "iostream"
#include <random>
#include <fstream>
#include "Eigen/Eigen"
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <thread>
#include "munkres.h"
#include "map_parameters.h"


static const int observation_pyramid_num_h = (int)half_fov_h * 2 / ANGLE_RESOLUTION;
static const int observation_pyramid_num_v = (int)half_fov_v * 2 / ANGLE_RESOLUTION;
static const int observation_pyramid_num = observation_pyramid_num_h * observation_pyramid_num_v;

static const int SAFE_PARTICLE_NUM = VOXEL_NUM * MAX_PARTICLE_NUM_VOXEL + 1e5;
static const int SAFE_PARTICLE_NUM_VOXEL = MAX_PARTICLE_NUM_VOXEL * 2;
static const int SAFE_PARTICLE_NUM_PYRAMID = SAFE_PARTICLE_NUM/PYRAMID_NUM * 2;

//An estimated number. If the observation points are too dense (over 100 points in one pyramid), the overflowed points will be ignored. It is suggested to use a voxel filter to the original point cloud.
static const int observation_max_points_num_one_pyramid = 100;
static const float obstacle_thickness_for_occlusion = 0.3;

//int velocity_estimation_error_occurred = 0;

#define GAUSSIAN_RANDOMS_NUM 1000000

#define O_MAKE_VALID 1       // use |= operator
#define O_MAKE_INVALID  0    // use &= operator

using namespace std;

/** Struct for an individual particle**/
struct Particle{
    float px;
    float py;
    float pz;
    float vx;
    float vy;
    float vz;
    float weight;
    int voxel_index;
};

struct ClusterFeature{
    float center_x = 0.f;
    float center_y = 0.f;
    float center_z = 0.f;
    int point_num = 0;
    int match_cluster_seq = -1;
    float vx = -10000.f;
    float vy = -10000.f;
    float vz = -10000.f;
    float v = 0.f;
    float intensity = 0.f;
};


// flag value 0: invalid, value 1: valid but not newborn, value 3: valid newborn 7: Recently predicted
/// Container for voxels h particles
//  1.flag 2.vx 3.vy 4.vz 5.px 6.py 7.pz
//  8.weight 9.update time
static float voxels_with_particle[VOXEL_NUM][SAFE_PARTICLE_NUM_VOXEL][9];

// 1. objects number; 2-4. Avg vx, vy, vz; 5-. Future objects number
static const int voxels_objects_number_dimension = 4 + PREDICTION_TIMES;
static float voxels_objects_number[VOXEL_NUM][voxels_objects_number_dimension];

/// Container for pyramids
// 0.flag 1.particle voxel index  2.particle index inside voxel
static int pyramids_in_fov[observation_pyramid_num][SAFE_PARTICLE_NUM_PYRAMID][3];

// 1.neighbors num 2-10:neighbor indexes
static int observation_pyramid_neighbors[observation_pyramid_num][10]{};

/// Variables for velocity estimation
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_current_view_rotated(new pcl::PointCloud<pcl::PointXYZ>());
static float current_position[3] = {0.f, 0.f, 0.f};
static float voxel_filtered_resolution = 0.15;
static float delt_t_from_last_observation = 0.f;
pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud_with_velocity(new pcl::PointCloud<pcl::PointXYZINormal>());


/** Storage for Gaussian randoms and Gaussian PDF**/
static float p_gaussian_randoms[GAUSSIAN_RANDOMS_NUM];
static float v_gaussian_randoms[GAUSSIAN_RANDOMS_NUM];
static float localization_gaussian_randoms[GAUSSIAN_RANDOMS_NUM];

static float standard_gaussian_pdf[20000];

class DSPMap{
public:

    DSPMap(int init_particle_num = 0, float init_weight=0.01f)
            : voxel_num_x(MAP_LENGTH_VOXEL_NUM),  /// Must be odd. voxel_resolution_set*voxel_num_x_set = map length
              voxel_num_y(MAP_WIDTH_VOXEL_NUM),  /// Must be odd. voxel_resolution_set*voxel_num_y_set = map width
              voxel_num_z(MAP_HEIGHT_VOXEL_NUM),  /// Must be odd. voxel_resolution_set*voxel_num_z_set = map height
              voxel_resolution(VOXEL_RESOLUTION),
              angle_resolution(ANGLE_RESOLUTION),  /// degree, should be completely divided by 360 degrees.
              max_particle_num_voxel(MAX_PARTICLE_NUM_VOXEL),
              velocity_gaussian_random_seq(0),
              position_gaussian_random_seq(0),
              localization_gaussian_random_seq(0),
              position_prediction_stddev(0.2f),
              velocity_prediction_stddev(0.1f),
              sigma_observation(0.2f),
              sigma_localization(0.f),
              kappa(0.01f),
              P_detection(0.95f),
              update_time(0.f),
              update_counter(0),
              expected_new_born_objects(0.f),
              new_born_particle_weight(0.04f),
              new_born_particle_number_each_point(20),
              if_record_particle_csv(0),
              record_time(1.f),
              new_born_each_object_weight(0.f),
              total_time(0.0),
              update_times(0)
    {
        setInitParameters();

        addRandomParticles(init_particle_num, init_weight);

        cout << "Map is ready to update!" << endl;
    }

    ~DSPMap(){
        cout << "\n See you ;)" <<endl;
    }

    int update(int point_cloud_num, int size_of_one_point, float *point_cloud_ptr,
                float sensor_px, float sensor_py, float sensor_pz, double time_stamp_second,
                float sensor_quaternion_w, float sensor_quaternion_x,
                float sensor_quaternion_y, float sensor_quaternion_z)  /// also requires point cloud and velocity
    {
        /** Get delt p **/
        static float sensor_px_last = sensor_px;
        static float sensor_py_last = sensor_py;
        static float sensor_pz_last = sensor_pz;
        static double time_stamp_second_last = time_stamp_second;

        // Check if the odometry data is invalid
        if(fabs(sensor_quaternion_w) > 1.001f || fabs(sensor_quaternion_x) > 1.001f || fabs(sensor_quaternion_y) > 1.001f || fabs(sensor_quaternion_z) > 1.001f){
            cout << "Invalid quaternion." <<endl;
            return 0;
        }

        float odom_delt_px = sensor_px - sensor_px_last;
        float odom_delt_py = sensor_py - sensor_py_last;
        float odom_delt_pz = sensor_pz - sensor_pz_last;
        auto delt_t = (float)(time_stamp_second - time_stamp_second_last);

        if(fabs(odom_delt_px) > 10.f || fabs(odom_delt_py) > 10.f || fabs(odom_delt_pz) > 10.f || delt_t < 0.f || delt_t > 10.f){
            cout << "!!! delt_t = "<< delt_t <<endl;
            cout << "!!! sensor_px_last = " << sensor_px_last << "sensor_px = " << sensor_px << " odom_delt_px="<<odom_delt_px<<endl;
            cout << "!!! sensor_py_last = " << sensor_py_last << "sensor_py = " << sensor_py << " odom_delt_py="<<odom_delt_py<<endl;
            return 0;
        }

        // clock_t start11, finish11;
        // start11 = clock();

        current_position[0] = sensor_px_last = sensor_px;
        current_position[1] = sensor_py_last = sensor_py;
        current_position[2] = sensor_pz_last = sensor_pz;
        time_stamp_second_last = time_stamp_second;

        delt_t_from_last_observation = delt_t;

        /** Update pyramid boundary planes' normal vectors' parameters **/


        sensor_rotation_quaternion[0] = sensor_quaternion_w;
        sensor_rotation_quaternion[1] = sensor_quaternion_x;
        sensor_rotation_quaternion[2] = sensor_quaternion_y;
        sensor_rotation_quaternion[3] = sensor_quaternion_z;

        for(int i=0; i<observation_pyramid_num_h+1; i++){
            rotateVectorByQuaternion(&pyramid_BPnorm_params_ori_h[i][0], sensor_rotation_quaternion, &pyramid_BPnorm_params_h[i][0]);
        }

        for(int j=0; j<observation_pyramid_num_v+1; j++){
            rotateVectorByQuaternion(&pyramid_BPnorm_params_ori_v[j][0], sensor_rotation_quaternion, &pyramid_BPnorm_params_v[j][0]);
        }

        /** Insert point cloud to observation storage **/
        for(int i=0; i< observation_pyramid_num; i++){  //Initialize point num in the storage
            observation_num_each_pyramid[i] = 0; //Set number of observations in each pyramid as zero in the beginning
            point_cloud_max_length[i] = -1.f;
        }

        cloud_in_current_view_rotated->clear();  // Clear first

        int iter_num = 0; // This is defined because one point has more than one float values. like px, py, pz
        int valid_points = 0; //NOTE: the number of valid points might be different from input points number is the real FOV is larger than the defined FOV. However, if the point is outside of the map but is still in FOV, it will be counted.
        for(int p_seq=0; p_seq<point_cloud_num; ++p_seq)
        {
            float rotated_point_this[3];
            rotateVectorByQuaternion(&point_cloud_ptr[iter_num], sensor_rotation_quaternion, rotated_point_this);

            // Store in pcl point cloud for velocity estimation of new born particles
            pcl::PointXYZ p_this;
            p_this.x = rotated_point_this[0];
            p_this.y = rotated_point_this[1];
            p_this.z = rotated_point_this[2];
            cloud_in_current_view_rotated->push_back(p_this);

            // Store in pyramids for update
            if(ifInPyramidsArea(rotated_point_this[0], rotated_point_this[1], rotated_point_this[2]))
            {
                int pyramid_index_h, pyramid_index_v;
                pyramid_index_h = findPointPyramidHorizontalIndex(rotated_point_this[0], rotated_point_this[1], rotated_point_this[2]);
                pyramid_index_v = findPointPyramidVerticalIndex(rotated_point_this[0], rotated_point_this[1], rotated_point_this[2]);

                int pyramid_index = pyramid_index_h * observation_pyramid_num_v + pyramid_index_v;
                int  observation_inner_seq = observation_num_each_pyramid[pyramid_index];

                float length = sqrtf( rotated_point_this[0]*rotated_point_this[0] + rotated_point_this[1]*rotated_point_this[1] + rotated_point_this[2]*rotated_point_this[2]);

                // get point cloud position in global coordinate
                point_cloud[pyramid_index][observation_inner_seq][0] = rotated_point_this[0];
                point_cloud[pyramid_index][observation_inner_seq][1] = rotated_point_this[1];
                point_cloud[pyramid_index][observation_inner_seq][2] = rotated_point_this[2];
                point_cloud[pyramid_index][observation_inner_seq][3] = 0.f;
                point_cloud[pyramid_index][observation_inner_seq][4] = length;

                if(point_cloud_max_length[pyramid_index] < length){  // to be used to judge if a particle is occluded
                    point_cloud_max_length[pyramid_index] = length;
                }

                observation_num_each_pyramid[pyramid_index] += 1;

                // Omit the overflowed observation points. It is suggested to used a voxel filter on the input point clouds to avoid overflow.
                if(observation_num_each_pyramid[pyramid_index] >= observation_max_points_num_one_pyramid){
                    observation_num_each_pyramid[pyramid_index] = observation_max_points_num_one_pyramid - 1;
                }

                ++ valid_points;
            }

            iter_num += size_of_one_point;
        }

        expected_new_born_objects = new_born_particle_weight * (float)valid_points * (float)new_born_particle_number_each_point;
        new_born_each_object_weight = new_born_particle_weight * (float)new_born_particle_number_each_point;


        /// Start a new thread for velocity estimation
        std::thread velocity_estimation(velocityEstimationThread);

        /*** Prediction ***/
//        clock_t start7, finish7;
//        start7 = clock();

        mapPrediction(-odom_delt_px, -odom_delt_py, -odom_delt_pz, delt_t);  // Particles move in the opposite of the robot moving direction
//
//        finish7 = clock();
//        double duration7= (double)(finish7 - start7) / CLOCKS_PER_SEC;
//        printf( "****** Prediction time %f seconds\n", duration7 );

        /*** Update ***/
//        clock_t start8, finish8;
//        start8 = clock();
        if(point_cloud_num >= 0){
            mapUpdate();
        }else{
            cout << "No points to update." <<endl;
        }

//        finish8 = clock();
//        double duration8= (double)(finish8 - start8) / CLOCKS_PER_SEC;
//        printf( "****** Update time %f seconds\n", duration8 );


        /** Wait until optical flow calculation is finished **/
        velocity_estimation.join();

        /** Add updated new born particles ***/
//        clock_t start1, finish1;
//        start1 = clock();

        if(point_cloud_num >= 0){
            mapAddNewBornParticlesByObservation();
        }

//        finish1 = clock();
//        double duration1 = (double)(finish1 - start1) / CLOCKS_PER_SEC;
//        printf( "****** new born time %f seconds\n", duration1);

        /** Calculate object number and Resample **/
//        clock_t start9, finish9;
//        start9 = clock();
        /// NOTE in this step the flag which is set to be 7.f in prediction step will be changed to 1.f or 0.6f.
        /// Removing this step will make prediction malfunction unless the flag is reset somewhere else.
        mapOccupancyCalculationAndResample();

//        finish9 = clock();
//        double duration9 = (double)(finish9 - start9) / CLOCKS_PER_SEC;
//        printf( "****** Resample time %f seconds\n", duration9);


        // finish11 = clock();
        // double duration11= (double)(finish11 - start11) / CLOCKS_PER_SEC;
        // printf( "****** Update time %f seconds\n", duration11 );

        // total_time += duration11;
        // ++ update_times;
        // printf( "****** Average update time = %lf seconds\n", total_time / double(update_times));



//        printf( "****** Total time %f seconds\n", duration1 + duration7 + duration8 + duration9 + duration11);
//        printf("############################## \n \n");

        /*** Record particles for analysis  ***/
        static int recorded_once_flag = 0;

        if(if_record_particle_csv){
            if(if_record_particle_csv < 0 || (update_time > record_time && !recorded_once_flag)){
                recorded_once_flag = 1;

                ofstream particle_log_writer;
                string file_name = "/home/clarence/particles_update_t_" + to_string(update_counter) + "_"+ to_string((int)(update_time*1000)) + ".csv";
                particle_log_writer.open(file_name, ios::out | ios::trunc);

                for(int i=0; i<voxels_total_num; i++){
                    for(int j=0; j<SAFE_PARTICLE_NUM_VOXEL; j++){
                        if(voxels_with_particle[i][j][0] > 0.1f){
                            for(int k=0; k<8; k++){
                                //  1.flag 2.vx 3.vy 4.vz 5.px 6.py 7.pz
                                //  8.weight 9.update time
                                particle_log_writer << voxels_with_particle[i][j][k] <<",";
                            }
                            particle_log_writer << i <<"\n";
                        }
                    }
                }
                particle_log_writer.close();
            }
        }

        return 1;
    }

    void setPredictionVariance(float p_stddev, float v_stddev){
        position_prediction_stddev = p_stddev;
        velocity_prediction_stddev = v_stddev;
        // regenerate randoms
        generateGaussianRandomsVectorZeroCenter();
    }

    void setObservationStdDev(float ob_stddev){
        sigma_observation = ob_stddev;
        sigma_update = sigma_observation;

        cout << "Observation stddev changed to " << sigma_observation << endl;
    }

    void setLocalizationStdDev(float lo_stddev){

#if(CONSIDER_LOCALIZATION_UNCERTAINTY)
        float tuned_scale_factor = 0.1f;
        sigma_localization = lo_stddev*tuned_scale_factor;
        cout << "Localization stddev changed to " << sigma_localization << endl;
        // regenerate randoms
        generateGaussianRandomsVectorZeroCenter();
#endif

    }

    void setNewBornParticleWeight(float weight){
        new_born_particle_weight = weight;
    }

    void setNewBornParticleNumberofEachPoint(int num){
        new_born_particle_number_each_point = num;
    }

    /// record_particle_flag O: don't record; -1 or other negative value: record all; positive value: record a time
    void setParticleRecordFlag(int record_particle_flag, float record_csv_time = 1.f){
        if_record_particle_csv = record_particle_flag;
        record_time =  record_csv_time;
    }

    static void setOriginalVoxelFilterResolution(float res){
        voxel_filtered_resolution = res;
    }


    void getOccupancyMap(int &obstacles_num, pcl::PointCloud<pcl::PointXYZ> &cloud, const float threshold=0.7){
        obstacles_num = 0;
        for(int i=0; i<voxels_total_num; i++){
            if(voxels_objects_number[i][0] > threshold){
                pcl::PointXYZ pcl_point;
                getVoxelPositionFromIndex(i, pcl_point.x, pcl_point.y, pcl_point.z);
                cloud.push_back(pcl_point);

                ++ obstacles_num;
            }

            /// Clear weights for next prediction
            for(int j=4; j<voxels_objects_number_dimension; ++j)
            {
                voxels_objects_number[i][j] = 0.f;
            }
        }
    }


    void getOccupancyMapWithVelocity(int &obstacles_num, std::vector<float> &weights, pcl::PointCloud<pcl::PointNormal> &cloud, const float threshold=0.7){
        obstacles_num = 0;
        for(int i=0; i<voxels_total_num; i++){
            if(voxels_objects_number[i][0] > threshold){
                pcl::PointNormal pcl_point;
                pcl_point.normal_x = voxels_objects_number[i][1]; //vx
                pcl_point.normal_y = voxels_objects_number[i][2]; //vy
                pcl_point.normal_z = voxels_objects_number[i][3]; //vz
                getVoxelPositionFromIndex(i, pcl_point.x, pcl_point.y, pcl_point.z);
                cloud.push_back(pcl_point);
                weights.push_back(voxels_objects_number[i][0]);
                ++ obstacles_num;
            }

            /// Clear weights for next prediction
            for(int j=4; j<voxels_objects_number_dimension; ++j)
            {
                voxels_objects_number[i][j] = 0.f;
            }
        }
    }


    void getOccupancyMapWithFutureStatus(int &obstacles_num, pcl::PointCloud<pcl::PointXYZ> &cloud, float *future_status, const float threshold=0.7){
        obstacles_num = 0;
        for(int i=0; i<voxels_total_num; i++){
            if(voxels_objects_number[i][0] > threshold){
                pcl::PointXYZ pcl_point;
                getVoxelPositionFromIndex(i, pcl_point.x, pcl_point.y, pcl_point.z);
                cloud.push_back(pcl_point);

                ++ obstacles_num;
            }

            for(int n=0; n < PREDICTION_TIMES; ++n){ // Set future weights
                *(future_status + i * PREDICTION_TIMES + n) = voxels_objects_number[i][n + 4];
            }

            /// Clear weights for next prediction
            for(int j=4; j<voxels_objects_number_dimension; ++j)
            {
                voxels_objects_number[i][j] = 0.f;
            }
        }
    }


    void getOccupancyMapWithRiskMaps(int &obstacles_num, pcl::PointCloud<pcl::PointXYZ> &cloud, float *risk_maps, const float threshold=0.7){
        obstacles_num = 0;
        for(int i=0; i<voxels_total_num; i++){
            if(voxels_objects_number[i][0] > threshold){
                pcl::PointXYZ pcl_point;
                getVoxelPositionFromIndex(i, pcl_point.x, pcl_point.y, pcl_point.z);
                cloud.push_back(pcl_point);

                ++ obstacles_num;
            }

            for(int n=0; n < RISK_MAP_NUMBER; ++n){ // Set future weights
                float temp_risk = 0.f;
                for(int m=0; m < RISK_MAP_PREDICTION_TIMES; ++m){
                    temp_risk += voxels_objects_number[i][4 + n*RISK_MAP_PREDICTION_TIMES +m];
                }
                *(risk_maps + i * RISK_MAP_NUMBER + n) = temp_risk;
            }

            /// Clear weights for next prediction
            for(int j=4; j<voxels_objects_number_dimension; ++j)
            {
                voxels_objects_number[i][j] = 0.f;
            }
        }
    }


    ///NOTE: If you don't want to use any visualization functions like "getOccupancyMap"
    ///      or "getOccupancyMapWithVelocity", you must call this function after update process.
    void clearOccupancyMapPrediction(){
        for(int i=0; i<voxels_total_num; i++){
            for(int j=4; j<voxels_objects_number_dimension; ++j)
            {
                voxels_objects_number[i][j] = 0.f;
            }
        }
    }

    /// Get clustered result for visualization
    void getKMClusterResult(pcl::PointCloud<pcl::PointXYZINormal> &cluster_cloud){
        for(auto &point : *input_cloud_with_velocity){
            cluster_cloud.push_back(point);
        }
    }


private:
    /** Parameters **/
    int voxel_num_x;
    int voxel_num_y;
    int voxel_num_z;
    float voxel_resolution;

    int pyramid_num_h;
    int pyramid_num_v;
    int angle_resolution;
    float angle_resolution_half;

    float angle_resolution_rad;
    float angle_resolution_rad_half;

    int voxels_total_num;
    int pyramid_total_num;

    float map_length_x_half; // real size, m
    float map_length_y_half; // real size, m
    float map_length_z_half; // real size, m

    int max_particle_num_voxel;

    float position_prediction_stddev;
    float velocity_prediction_stddev;

    float sigma_observation;
    float sigma_localization;
    float sigma_update;

    float P_detection;

    int if_record_particle_csv;
    float record_time;

    /** Variables **/
    int position_gaussian_random_seq;
    int localization_gaussian_random_seq;
    int velocity_gaussian_random_seq;

    float kappa;

    float update_time;
    int update_counter;

    float expected_new_born_objects;

    float new_born_particle_weight;
    int new_born_particle_number_each_point;
    float new_born_each_object_weight;

    // 1.px, 2.py, 3.pz 4.acc 5.length for later usage
    float point_cloud[observation_pyramid_num][observation_max_points_num_one_pyramid][5];

    // 1.point_num
    int observation_num_each_pyramid[observation_pyramid_num]{};

    float sensor_rotation_quaternion[4];

    // Normal vectors for pyramids boundary planes when sensor has no rotation
    float pyramid_BPnorm_params_ori_h[observation_pyramid_num_h+1][3]; // x, y, z
    float pyramid_BPnorm_params_ori_v[observation_pyramid_num_v+1][3];

    // Normal vectors for pyramids boundary planes when sensor rotated
    float pyramid_BPnorm_params_h[observation_pyramid_num_h+1][3];
    float pyramid_BPnorm_params_v[observation_pyramid_num_v+1][3];


    // Max length, used to judge if occlusion happens
    float point_cloud_max_length[observation_pyramid_num];

    float half_fov_h_rad;
    float half_fov_v_rad;

    double total_time;
    unsigned int update_times;


private:
    void setInitParameters(){

        /*** Set parameters **/
        sigma_update = sigma_observation;

        map_length_x_half = (voxel_resolution* (float)voxel_num_x) * 0.5f;
        map_length_y_half = (voxel_resolution* (float)voxel_num_y) * 0.5f;
        map_length_z_half = (voxel_resolution* (float)voxel_num_z) * 0.5f;

        voxels_total_num = VOXEL_NUM;

        pyramid_num_h = 360 / angle_resolution;
        pyramid_num_v = 180 / angle_resolution;
        pyramid_total_num = PYRAMID_NUM;

        half_fov_h_rad = (float)half_fov_h / 180.f * M_PIf32;
        half_fov_v_rad = (float)half_fov_v / 180.f * M_PIf32;

        angle_resolution_half = (float)angle_resolution /  2.f;

        angle_resolution_rad = (float)angle_resolution / 180.f * M_PIf32;
        angle_resolution_rad_half = angle_resolution_rad/2.f;

        // Initialize voxels
        for(auto & i : voxels_with_particle){
            for(auto & j : i){
                for(float & k : j){
                    k = 0.f;
                }
            }
        }

        // Initialize pyramids
        for(auto & pyramid : pyramids_in_fov){
            for(auto & p : pyramid){
                p[0] = 0;
                p[1] = 0;
            }
        }

        /// New: set pyramid plane initial parameters
        int h_start_seq = - half_fov_h / angle_resolution;
        int h_end_seq = -h_start_seq;
        for(int i=h_start_seq; i<=h_end_seq; i++){
            pyramid_BPnorm_params_ori_h[i+h_end_seq][0] = -sin((float)i*angle_resolution_rad); // x
            pyramid_BPnorm_params_ori_h[i+h_end_seq][1] = cos((float)i*angle_resolution_rad); // y
            pyramid_BPnorm_params_ori_h[i+h_end_seq][2] = 0.f; // z
        }

        int v_start_seq = -half_fov_v / angle_resolution;
        int v_end_seq = -v_start_seq;
        for(int i=v_start_seq; i<=v_end_seq; i++){
            pyramid_BPnorm_params_ori_v[i+v_end_seq][0] = sin((float)i*angle_resolution_rad);  // x
            pyramid_BPnorm_params_ori_v[i+v_end_seq][1] = 0.f; // y
            pyramid_BPnorm_params_ori_v[i+v_end_seq][2] = cos((float)i*angle_resolution_rad); // z
        }

        // Find neighborhood pyramids' indexes for observation pyramids
        for(int i=0; i< observation_pyramid_num; i++){  //Initialize point num in the storage
            findPyramidNeighborIndexInFOV(i, observation_pyramid_neighbors[i][0], &observation_pyramid_neighbors[i][1]);
        }

        // Generate Gaussian randoms.
        srand (static_cast <unsigned> (time(0))); //TEST
        generateGaussianRandomsVectorZeroCenter();
        calculateNormalPDFBuffer();


    }


    void addRandomParticles(int particle_num, float avg_weight)
    {
        /*** Initialize some particles ***/
        int successfully_added_num = 0;
        int voxel_overflow_num = 0;

        for(int i=0; i<particle_num; i++) {
            std::shared_ptr<Particle> particle_ptr{new Particle};

            particle_ptr->px = generateRandomFloat(-map_length_x_half, map_length_x_half);
            particle_ptr->py = generateRandomFloat(-map_length_y_half, map_length_y_half);
            particle_ptr->pz = generateRandomFloat(-map_length_z_half, map_length_z_half);
            particle_ptr->vx = generateRandomFloat(-1.f, 1.f);
            particle_ptr->vy = generateRandomFloat(-1.f, 1.f);
            particle_ptr->vz = generateRandomFloat(-1.f, 1.f);
            particle_ptr->weight = avg_weight;

            if (getParticleVoxelsIndex(*particle_ptr, particle_ptr->voxel_index)) {

                int test = addAParticle(*particle_ptr, particle_ptr->voxel_index);
                if(test>0){
                    successfully_added_num ++;
                }else{
                    voxel_overflow_num ++;
                }
            }
        }

//        cout << "successfully_added_num="<<successfully_added_num<<endl;
//        cout << "voxel_overflow_num="<<voxel_overflow_num<<endl;
    }


    void mapPrediction(float odom_delt_px, float odom_delt_py, float odom_delt_pz, float delt_t)
    {
        int operation_counter = 0;
        int exist_particles = 0;
        int voxel_full_remove_counter = 0, pyramid_full_remove_counter = 0;
        int moves_out_counter = 0;

        update_time += delt_t;
        update_counter += 1;

        /// Clear pyramids first
        for(auto & j : pyramids_in_fov){
            for(auto & i : j){
                i[0] &= O_MAKE_INVALID;
            }
        }

        /// Update Particles' state and index in both voxels and pyramids
        for(int v_index=0; v_index<VOXEL_NUM; ++v_index)
        {
            for(int p=0; p<SAFE_PARTICLE_NUM_VOXEL; p++)
            {
                if(voxels_with_particle[v_index][p][0] >0.1f && voxels_with_particle[v_index][p][0] <6.f){  /// exsit, but not new moved
                    voxels_with_particle[v_index][p][0] = 1.f; // If valid, remove resample flag.
                    ++ operation_counter;

                    if(fabs(voxels_with_particle[v_index][p][1]*voxels_with_particle[v_index][p][2]*voxels_with_particle[v_index][p][3]) < 1e-6){
                        // keep small, for static obstacles
                    }else{
                        voxels_with_particle[v_index][p][1] += getVelocityGaussianZeroCenter();  //vx
                        voxels_with_particle[v_index][p][2] += getVelocityGaussianZeroCenter();  //vy
                        voxels_with_particle[v_index][p][3] += getVelocityGaussianZeroCenter();  //vz
                    }

#if(LIMIT_MOVEMENT_IN_XY_PLANE)
                    voxels_with_particle[v_index][p][3] = 0.f;
#endif



#if(CONSIDER_LOCALIZATION_UNCERTAINTY)
                    voxels_with_particle[v_index][p][4] += delt_t*voxels_with_particle[v_index][p][1] + odom_delt_px + getLocalizationGaussianZeroCenter();
                    voxels_with_particle[v_index][p][5] += delt_t*voxels_with_particle[v_index][p][2] + odom_delt_py + getLocalizationGaussianZeroCenter();
                    voxels_with_particle[v_index][p][6] += delt_t*voxels_with_particle[v_index][p][3] + odom_delt_pz + getLocalizationGaussianZeroCenter();
#else
                    voxels_with_particle[v_index][p][4] += delt_t*voxels_with_particle[v_index][p][1] + odom_delt_px;  //px
                    voxels_with_particle[v_index][p][5] += delt_t*voxels_with_particle[v_index][p][2] + odom_delt_py;  //py
                    voxels_with_particle[v_index][p][6] += delt_t*voxels_with_particle[v_index][p][3] + odom_delt_pz;  //pz
#endif


                    int particle_voxel_index_new;
                    if(getParticleVoxelsIndex(voxels_with_particle[v_index][p][4], voxels_with_particle[v_index][p][5],
                                              voxels_with_particle[v_index][p][6], particle_voxel_index_new))
                    {
                        // move particle. If moved, the flag turns to 7.f. If should move but failed because target voxel is full, delete the voxel.
                        int move_flag = moveParticle(particle_voxel_index_new, v_index, p, &voxels_with_particle[v_index][p][0]);
                        if(move_flag == -2){
                            // Move the particle, if fails, "moveParticleByVoxel" will delete the particle
                            ++ pyramid_full_remove_counter;
                            continue;
                        }else if(move_flag == -1){
                            ++ voxel_full_remove_counter;
                            continue;
                        }
                        ++ exist_particles;

                    }
                    else{
                        /// Particle moves out
                        removeParticle(&voxels_with_particle[v_index][p][0]);
                        ++ moves_out_counter;
                    }

                }
            }
        }

//        cout << "exist_particles=" << exist_particles<<endl;
//        cout << "voxel_full_remove_counter="<<voxel_full_remove_counter<<endl;
//        cout << "pyramid_full_remove_counter="<<pyramid_full_remove_counter<<endl;
//        cout << "moves_out_counter="<<moves_out_counter<<endl;
//        cout << "operation_counter_prediction="<<operation_counter<<endl;

        if(moves_out_counter > 10000){
            cout <<"!!!!! An error occured! delt_t = " << delt_t <<endl;
            cout << "odom_delt_px = " << odom_delt_px <<" odom_delt_py = " << odom_delt_py << "odom_delt_pz=" << odom_delt_pz<< endl;
        }

    }


    void mapUpdate()
    {
        int operation_counter_update = 0;

        /// Calculate Ck + kappa first
        for(int i=0; i<observation_pyramid_num; ++i){
            for(int j=0; j<observation_num_each_pyramid[i]; ++j){
                // Iteration of z
                for(int n_seq=0; n_seq<observation_pyramid_neighbors[i][0]; ++n_seq){
                    int pyramid_check_index = observation_pyramid_neighbors[i][n_seq+1]; //0.neighbors num 1-9:neighbor indexes

                    for(int particle_seq=0; particle_seq<SAFE_PARTICLE_NUM_PYRAMID; ++particle_seq){
                        if(pyramids_in_fov[pyramid_check_index][particle_seq][0] & O_MAKE_VALID){  //Check only valid particles
                            int particle_voxel_index = pyramids_in_fov[pyramid_check_index][particle_seq][1];
                            int particle_voxel_inner_index = pyramids_in_fov[pyramid_check_index][particle_seq][2];

//                            cout<<"pyramid_check_index="<<pyramid_check_index<<", particle_v_inner_index="<<particle_v_inner_index<<", point_cloud[i][j][0]="<<point_cloud[i][j][0]<<endl;

                            float gk = queryNormalPDF(
                                    voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][4],
                                    point_cloud[i][j][0], sigma_update)
                                       * queryNormalPDF(
                                    voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][5],
                                    point_cloud[i][j][1], sigma_update)
                                       * queryNormalPDF(
                                    voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][6],
                                    point_cloud[i][j][2], sigma_update);

                            point_cloud[i][j][3] += P_detection * voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][7] * gk;
                        }
                    }
                }
                /// add weight for new born particles
                point_cloud[i][j][3] += (expected_new_born_objects + kappa);
            }
        }


        /// Update weight for each particle in view
        for(int i=0; i < observation_pyramid_num; i++)
        {
            int current_pyramid_index = i;

            for(int inner_seq=0; inner_seq<SAFE_PARTICLE_NUM_PYRAMID; inner_seq++){
                // Iteration of particles
                if(pyramids_in_fov[current_pyramid_index][inner_seq][0] & O_MAKE_VALID){ //update only valid particle

                    int neighbor_num = observation_pyramid_neighbors[current_pyramid_index][0];

                    int particle_voxel_index = pyramids_in_fov[current_pyramid_index][inner_seq][1];
                    int particle_voxel_inner_index = pyramids_in_fov[current_pyramid_index][inner_seq][2];

                    float px_this = voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][4];
                    float py_this = voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][5];
                    float pz_this = voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][6];
                    float particle_dist_length = sqrtf(px_this*px_this + py_this*py_this + pz_this*pz_this);

                    if(point_cloud_max_length[i] > 0.f && particle_dist_length > point_cloud_max_length[i] + obstacle_thickness_for_occlusion) //Update only particles that are not occluded, use voxel_resolution as the distance metric.
                    {
                        // occluded
                        continue;
                    }


                    float sum_by_zk = 0.f;
                    for(int neighbor_seq=0; neighbor_seq<neighbor_num; ++neighbor_seq)
                    {
                        int neighbor_index = observation_pyramid_neighbors[current_pyramid_index][neighbor_seq+1];

                        for(int z_seq=0; z_seq<observation_num_each_pyramid[neighbor_index]; ++z_seq) //for all observation points in a neighbor pyramid
                        {
                            float gk = queryNormalPDF(px_this, point_cloud[neighbor_index][z_seq][0], sigma_update)
                                       * queryNormalPDF(py_this, point_cloud[neighbor_index][z_seq][1], sigma_update)
                                       * queryNormalPDF(pz_this, point_cloud[neighbor_index][z_seq][2], sigma_update);

                            sum_by_zk += P_detection * gk / point_cloud[neighbor_index][z_seq][3];
                            ++operation_counter_update;
                        }


                    }

                    voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][7] *= ((1 - P_detection) + sum_by_zk);
                    voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][8] = update_time;
                }
            }
        }
//        cout << "operation_counter_update=" << operation_counter_update <<endl;

    }

public:
    void mapAddNewBornParticlesByObservation()
    {
        /** Calculate normalization coefficient first **/
        float normalization_coefficient = 0.f;
        for(int i=0; i< observation_pyramid_num; i++){
            for(int j=0; j< observation_num_each_pyramid[i]; j++){
                normalization_coefficient += 1.f / point_cloud[i][j][3];
            }
        }
        float updated_weight_new_born = new_born_particle_weight * normalization_coefficient;

        /** Add new born particles **/
        static int min_static_new_born_particle_number_each_point = (int)((float)new_born_particle_number_each_point * 0.15f);
        static int static_new_born_particle_number_each_point = (int)((float)new_born_particle_number_each_point * 0.4f);  // static points takes 3 in 10
        static int pf_derive_new_born_particle_number_each_point = (int)((float)new_born_particle_number_each_point * 0.5f); // Derived takes half
        static const int model_generated_particle_number_each_point = (int)((float)new_born_particle_number_each_point * 0.8f);

        int successfully_born_particles = 0;
        /// TODO: Improve efficiency in this new born procedure
        for(auto & point : *input_cloud_with_velocity)
        {
            pcl::PointXYZ p_corrected;
            p_corrected.x = point.x - current_position[0];
            p_corrected.y = point.y - current_position[1];
            p_corrected.z = point.z - current_position[2];

            int point_voxel_index;
            float static_particle_weight_sum = 0.f;
            float dynamic_particle_weight_sum = 0.f;
            float static_or_dynamic_weight_sum = 0.f;

            if(getParticleVoxelsIndex(p_corrected.x, p_corrected.y, p_corrected.z, point_voxel_index)){
                //This condition should always be true because the point cloud outside of the map should be omitted in the first place. Just an insurance.
                for(int kk=0; kk<SAFE_PARTICLE_NUM_VOXEL; ++kk){
                    if(voxels_with_particle[point_voxel_index][kk][0] > 0.9f && voxels_with_particle[point_voxel_index][kk][0] < 14.f){ //not new born
                        float v_abs = fabs(voxels_with_particle[point_voxel_index][kk][1]) + fabs(voxels_with_particle[point_voxel_index][kk][2]) + fabs(voxels_with_particle[point_voxel_index][kk][3]);
                        if(v_abs < 0.1f){
                            // Static
                            static_particle_weight_sum += voxels_with_particle[point_voxel_index][kk][7];
                        }else if(v_abs < 0.5f){
                            // Static or dynamic
                            static_or_dynamic_weight_sum += voxels_with_particle[point_voxel_index][kk][7];
                        }
                        else{
                            //Dynamic
                            dynamic_particle_weight_sum += voxels_with_particle[point_voxel_index][kk][7];
                        }
                    }
                }
            }
            else{
                continue;
            }

            // Dempster-Shafer Theory
            float total_weight_voxel = static_particle_weight_sum + dynamic_particle_weight_sum + static_or_dynamic_weight_sum;
            float m_static = static_particle_weight_sum / total_weight_voxel;
            float m_dynamic = dynamic_particle_weight_sum / total_weight_voxel;
            float m_static_or_dynamic = static_or_dynamic_weight_sum / total_weight_voxel;

            float p_static = (m_static + m_static + m_static_or_dynamic) * 0.5f;
            float p_dynamic = (m_dynamic + m_dynamic + m_static_or_dynamic) * 0.5f;
            float normalization_p = p_static + p_dynamic;
            float p_static_normalized = p_static / normalization_p;
            float p_dynamic_normalized = p_dynamic / normalization_p;

            static_new_born_particle_number_each_point = (int)((float)model_generated_particle_number_each_point * p_static_normalized);
            pf_derive_new_born_particle_number_each_point = model_generated_particle_number_each_point - static_new_born_particle_number_each_point;

            // Set a minimum number of static particles
            static_new_born_particle_number_each_point = max(min_static_new_born_particle_number_each_point, static_new_born_particle_number_each_point);

            for(int p=0; p<new_born_particle_number_each_point; p++){
                std::shared_ptr<Particle> particle_ptr{new Particle};

                particle_ptr->px = p_corrected.x + getPositionGaussianZeroCenter();
                particle_ptr->py = p_corrected.y + getPositionGaussianZeroCenter();
                particle_ptr->pz = p_corrected.z + getPositionGaussianZeroCenter();

                if (getParticleVoxelsIndex(*particle_ptr, particle_ptr->voxel_index)) {
                    // Particle index might be different from the point index because a random Gaussian is added.
                    if(p < static_new_born_particle_number_each_point){  // add static points
                        particle_ptr->vx = 0.f;
                        particle_ptr->vy = 0.f;
                        particle_ptr->vz = 0.f;
                    }else if(point.normal_x > -100.f && p < model_generated_particle_number_each_point){ //p < pf_derive_new_born_particle_number_each_point + static_new_born_particle_number_each_point){
                        /// Use estimated velocity to generate new particles
                        if(point.intensity > 0.01f){
                            particle_ptr->vx = point.normal_x + 4*getVelocityGaussianZeroCenter();
                            particle_ptr->vy = point.normal_y + 4*getVelocityGaussianZeroCenter();
                            particle_ptr->vz = point.normal_z + 4*getVelocityGaussianZeroCenter();
                        }else{ //static points like ground
                            particle_ptr->vx = 0.f;
                            particle_ptr->vy = 0.f;
                            particle_ptr->vz = 0.f;
                        }
                    }
                    else{ /// Considering Random Noise
                        if(point.intensity > 0.01f){
                            particle_ptr->vx = generateRandomFloat(-1.5f, 1.5f);
                            particle_ptr->vy = generateRandomFloat(-1.5f, 1.5f);
                            particle_ptr->vz = generateRandomFloat(-0.5f, 0.5f);
                        }else{ //static points like ground
                            particle_ptr->vx = 0.f;
                            particle_ptr->vy = 0.f;
                            particle_ptr->vz = 0.f;
                        }
                    }

#if(LIMIT_MOVEMENT_IN_XY_PLANE)
                    particle_ptr->vz = 0.f;
#endif

                    particle_ptr->weight = updated_weight_new_born;

                    int test = addAParticle(*particle_ptr, particle_ptr->voxel_index);
                    if(test>0){
                        ++ successfully_born_particles;
                    }
                }
            }
        }


//        cout << "successfully_born_particles = "<<successfully_born_particles<<endl;
    }

private:
    void mapOccupancyCalculationAndResample()
    {
        int removed_particle_counter = 0;
        int particle_num_after_resampling_should_be = 0;

        for(int v_index=0; v_index<VOXEL_NUM; ++v_index)
        {
            // Calculate estimated object number in each voxel
            static float weight_sum_voxel, vx_sum_voxel, vy_sum_voxel, vz_sum_voxel;
            weight_sum_voxel = 0.f;
            vx_sum_voxel = vy_sum_voxel = vz_sum_voxel = 0.f;

            int particle_num_voxel = 0;
            int old_particle_num_voxel = 0;
            for(int p=0; p<SAFE_PARTICLE_NUM_VOXEL; p++)
            {
                if(voxels_with_particle[v_index][p][0] > 0.1f){
                    if(voxels_with_particle[v_index][p][7] < 1e-3){
                        voxels_with_particle[v_index][p][0] = 0.f;  // Remove the particle directly if the weight is too small
                    }else{
                        if(voxels_with_particle[v_index][p][0] < 10.f){  //exclude new-born particles
                            ++old_particle_num_voxel;
                            vx_sum_voxel += voxels_with_particle[v_index][p][1];
                            vy_sum_voxel += voxels_with_particle[v_index][p][2];
                            vz_sum_voxel += voxels_with_particle[v_index][p][3];

                            /*** Future status prediction ***/
                            float px_future, py_future, pz_future;
                            for(int times = 0; times < PREDICTION_TIMES; ++times)
                            {
                                float prediction_time = prediction_future_time[times];
                                px_future = voxels_with_particle[v_index][p][4] + voxels_with_particle[v_index][p][1] * prediction_time;
                                py_future = voxels_with_particle[v_index][p][5] + voxels_with_particle[v_index][p][2] * prediction_time;
                                pz_future = voxels_with_particle[v_index][p][6] + voxels_with_particle[v_index][p][3] * prediction_time;

                                int prediction_index;
                                if(getParticleVoxelsIndex(px_future, py_future, pz_future, prediction_index)) {
                                    voxels_objects_number[prediction_index][4+times] += voxels_with_particle[v_index][p][7]; //weight
                                }
                            }
                            /**** End of prediction ****/

                        }

                        voxels_with_particle[v_index][p][0] = 1.f; // Remove newborn flag and moved flag in prediction
                        ++particle_num_voxel;
                        weight_sum_voxel += voxels_with_particle[v_index][p][7];
                    }
                }
            }
            voxels_objects_number[v_index][0] = weight_sum_voxel;

            if(old_particle_num_voxel > 0){
                voxels_objects_number[v_index][1] = vx_sum_voxel / (float)old_particle_num_voxel;
                voxels_objects_number[v_index][2] = vy_sum_voxel / (float)old_particle_num_voxel;
                voxels_objects_number[v_index][3] = vz_sum_voxel / (float)old_particle_num_voxel;
            }else{
                voxels_objects_number[v_index][1] = 0.f;
                voxels_objects_number[v_index][2] = 0.f;
                voxels_objects_number[v_index][3] = 0.f;
            }

            if(particle_num_voxel < 5){  //Too few particles, no need to resample.
                particle_num_after_resampling_should_be += particle_num_voxel; //for test
                continue;
            }

            // Calculate desired particle number after resampling
            int particle_num_voxel_after_resample;
            if(particle_num_voxel > MAX_PARTICLE_NUM_VOXEL){
                particle_num_voxel_after_resample = MAX_PARTICLE_NUM_VOXEL;
            }else{
                particle_num_voxel_after_resample = particle_num_voxel;
            }

            static float weight_after_resample;
            weight_after_resample = weight_sum_voxel / (float)particle_num_voxel_after_resample;

            particle_num_after_resampling_should_be += particle_num_voxel_after_resample;

            // Resample
            float acc_ori_weight = 0.f;
            float acc_new_weight = weight_after_resample * 0.5f;
            for(int p=0; p<SAFE_PARTICLE_NUM_VOXEL; ++p)
            {
                if(voxels_with_particle[v_index][p][0] > 0.7f){ //exclude invalid and newly_added_by_resampling particles
                    float ori_particle_weight = voxels_with_particle[v_index][p][7];
                    acc_ori_weight += ori_particle_weight;

                    if(acc_ori_weight > acc_new_weight){
                        voxels_with_particle[v_index][p][7] = weight_after_resample; // keep the particle but change weight
                        acc_new_weight += weight_after_resample;

                        int if_space_is_currently_full = 0;
                        /** copy particles that have a very large weight **/
                        int p_i=0;

                        while(acc_ori_weight > acc_new_weight){ // copy the particle if the original weight is very large
                            int if_found_position_in_voxel = 0;
                            if(!if_space_is_currently_full){
                                for( ; p_i<SAFE_PARTICLE_NUM_VOXEL; ++p_i){
                                    if(voxels_with_particle[v_index][p_i][0] < 0.1f){ // find an empty position in voxel
                                        // Now copy the particle
                                        voxels_with_particle[v_index][p_i][0] = 0.6f; // Flag: newly_added_by_resampling
                                        for(int k=1; k<9; k++){
                                            voxels_with_particle[v_index][p_i][k] = voxels_with_particle[v_index][p][k];
                                        }
                                        if_found_position_in_voxel = 1;
                                        break;
                                    }
                                }
                            }

                            if(!if_found_position_in_voxel){
                                // If the particle should be copied but no space left in either voxel or pyramid, add the weight of the original particle to keep the total weight unchanged.
                                voxels_with_particle[v_index][p][7] += weight_after_resample;
                                if_space_is_currently_full = 1;
                            }

                            acc_new_weight += weight_after_resample;
                        }

                    }else{
                        // Remove the particle
                        voxels_with_particle[v_index][p][0] = 0.f;
                        removed_particle_counter ++;
                    }
                }

            }

        }

//        // Resampling result analysis
//        int particle_num_after_resampling = 0;
//
//        for(int v_index=0; v_index<VOXEL_NUM; ++v_index)
//        {
//            for(int i=0; i<SAFE_PARTICLE_NUM_VOXEL; i++){
//                if(voxels_with_particle[v_index][i][0] > 0.1f){
//                    ++ particle_num_after_resampling;
//                }
//            }
//        }
//        cout <<"particle_num_after_resampling_should_be="<<particle_num_after_resampling_should_be<<endl;
//        cout <<"particle_num_after_resampling="<<particle_num_after_resampling<<endl;
//        cout <<"removed_particle_counter="<<removed_particle_counter<<endl;
    }


private: /*** Some specific functions ***/

    int getParticleVoxelsIndex(const Particle &p, int &index){
        if(ifParticleIsOut(p)){return 0;}
        auto x = (int)((p.px + map_length_x_half) / voxel_resolution);
        auto y = (int)((p.py + map_length_y_half) / voxel_resolution);
        auto z = (int)((p.pz + map_length_z_half) / voxel_resolution);
        index = z*voxel_num_y*voxel_num_x + y*voxel_num_x + x;

        if(index<0 || index>=VOXEL_NUM){
            return 0;
        }

        return 1;
    }

    int getParticleVoxelsIndex(const float &px, const float &py, const float &pz, int & index){
        if(ifParticleIsOut(px, py, pz)) {return 0;}
        auto x = (int)((px + map_length_x_half) / voxel_resolution);
        auto y = (int)((py + map_length_y_half) / voxel_resolution);
        auto z = (int)((pz + map_length_z_half) / voxel_resolution);
        index = z*voxel_num_y*voxel_num_x + y*voxel_num_x + x;

        if(index<0 || index>=VOXEL_NUM){
            return 0;
        }

        return 1;
    }

    void getVoxelPositionFromIndex(const int &index, float &px, float &py, float &pz) const{
        static const int z_change_storage_taken = voxel_num_y*voxel_num_x;
        static const int y_change_storage_taken = voxel_num_x;

        int z_index = index / z_change_storage_taken;
        int yx_left_indexes = index - z_index * z_change_storage_taken;
        int y_index = yx_left_indexes / y_change_storage_taken;
        int x_index = yx_left_indexes - y_index * y_change_storage_taken;


        static const float correction_x = -map_length_x_half + voxel_resolution*0.5f;
        static const float correction_y = -map_length_y_half + voxel_resolution*0.5f;
        static const float correction_z = -map_length_z_half + voxel_resolution*0.5f;

        px = (float)x_index * voxel_resolution + correction_x;
        py = (float)y_index * voxel_resolution + correction_y;
        pz = (float)z_index * voxel_resolution + correction_z;
    }

    int ifParticleIsOut(const Particle &p) const{
        if(p.px >= map_length_x_half || p.px <= -map_length_x_half ||
           p.py >= map_length_y_half || p.py <= -map_length_y_half ||
           p.pz >= map_length_z_half || p.pz <= -map_length_z_half){
            return 1;
        }
        else return 0;
    }

    int ifParticleIsOut(const float &px, const float &py, const float &pz) const{
        if(px >= map_length_x_half || px <= -map_length_x_half ||
           py >= map_length_y_half || py <= -map_length_y_half ||
           pz >= map_length_z_half || pz <= -map_length_z_half){
            return 1;
        }
        else return 0;
    }


    static void findPyramidNeighborIndexInFOV(const int &index_ori, int &neighbor_spaces_num, int *neighbor_spaces_index)
    {
        int h_index_ori = index_ori / observation_pyramid_num_v;
        int v_index_ori = index_ori % observation_pyramid_num_v;

        neighbor_spaces_num = 0;

        for(int i=-1; i<=1; ++i){
            for(int j=-1; j<=1; ++j){
                int h = h_index_ori + i;
                int v = v_index_ori + j;
                if(h>=0 && h<observation_pyramid_num_h && v>=0 && v<observation_pyramid_num_v)
                {
                    *(neighbor_spaces_index + neighbor_spaces_num) = h*observation_pyramid_num_v + v;
                    ++ neighbor_spaces_num;
                }
            }
        }

    }


    void generateGaussianRandomsVectorZeroCenter() const{
        std::default_random_engine random(time(NULL));
        std::normal_distribution<float> n1(0, position_prediction_stddev);
        std::normal_distribution<float> n2(0, velocity_prediction_stddev);
        std::normal_distribution<float> n3(0, sigma_localization);

        for(int i=0; i<GAUSSIAN_RANDOMS_NUM; i++){
            *(p_gaussian_randoms+i) = n1(random);
            *(v_gaussian_randoms+i) = n2(random);
            *(localization_gaussian_randoms+i) = n3(random);
        }

    }

    float getPositionGaussianZeroCenter(){
        float delt_p = p_gaussian_randoms[position_gaussian_random_seq];
        position_gaussian_random_seq += 1;
        if(position_gaussian_random_seq >= GAUSSIAN_RANDOMS_NUM){
            position_gaussian_random_seq = 0;
        }
        return delt_p;
    }


    float getLocalizationGaussianZeroCenter(){
        float delt_p = localization_gaussian_randoms[localization_gaussian_random_seq];
        localization_gaussian_random_seq += 1;
        if(localization_gaussian_random_seq >= GAUSSIAN_RANDOMS_NUM){
            localization_gaussian_random_seq = 0;
        }
        return delt_p;
    }

    float getVelocityGaussianZeroCenter(){
        float delt_v = v_gaussian_randoms[velocity_gaussian_random_seq];
        velocity_gaussian_random_seq += 1;
        if(velocity_gaussian_random_seq >= GAUSSIAN_RANDOMS_NUM){
            velocity_gaussian_random_seq = 0;
        }
        return delt_v;
    }

    /***
     * Return 0 if move operation fails, otherwise return 1.
     * **/
     int addAParticle(const Particle &p, const int &voxel_index) const{
        for(int i=0;i<SAFE_PARTICLE_NUM_VOXEL; i++){
            if(voxels_with_particle[voxel_index][i][0] < 0.1f){ // found an empty particle position
                voxels_with_particle[voxel_index][i][0] = 15.f;  //New born flag
                voxels_with_particle[voxel_index][i][1] = p.vx;
                voxels_with_particle[voxel_index][i][2] = p.vy;
                voxels_with_particle[voxel_index][i][3] = p.vz;
                voxels_with_particle[voxel_index][i][4] = p.px;
                voxels_with_particle[voxel_index][i][5] = p.py;
                voxels_with_particle[voxel_index][i][6] = p.pz;
                voxels_with_particle[voxel_index][i][7] = p.weight;
                voxels_with_particle[voxel_index][i][8] = update_time;

                return 1;
            }
        } /// If no space. Omit this particle in voxel

        return 0;
    }

    /***
     * Return 0 if move operation fails, otherwise return 1.
     * **/
    int moveParticle(const int& new_voxel_index, const int& current_v_index, const int& current_v_inner_index, float *ori_particle_flag_ptr)
    {
        int new_voxel_inner_index = current_v_inner_index;
        if(new_voxel_index != current_v_index){
            *ori_particle_flag_ptr = 0.f; // Remove from ori voxel first

            /// Find a space in the new voxel and then pyramid. If no space in either voxel or pyramid. This particle would vanish.
            int successfully_moved_by_voxel = 0;
            for(int i=0; i<SAFE_PARTICLE_NUM_VOXEL; ++i){
                if(voxels_with_particle[new_voxel_index][i][0] < 0.1f){ //empty
                    new_voxel_inner_index = i;
                    successfully_moved_by_voxel = 1;

                    voxels_with_particle[new_voxel_index][i][0] = 7.f;  //newly moved flag
                    for(int k=1; k<9; ++k){  // set v, p, weight, update time
                        voxels_with_particle[new_voxel_index][i][k] = *(ori_particle_flag_ptr+k);
                    }
                    break; ///Important
                }
            }

            if(!successfully_moved_by_voxel){  ///let the particle vanish
                return -1;
            }
        }

        // Now check pyramid, pyramids are cleared first so the particle in FOV must be added unless full.
        if(ifInPyramidsArea(voxels_with_particle[new_voxel_index][new_voxel_inner_index][4], voxels_with_particle[new_voxel_index][new_voxel_inner_index][5],
                            voxels_with_particle[new_voxel_index][new_voxel_inner_index][6]))
        {

            int h_index = findPointPyramidHorizontalIndex(voxels_with_particle[new_voxel_index][new_voxel_inner_index][4], voxels_with_particle[new_voxel_index][new_voxel_inner_index][5],
                                                          voxels_with_particle[new_voxel_index][new_voxel_inner_index][6]);

            int v_index = findPointPyramidVerticalIndex(voxels_with_particle[new_voxel_index][new_voxel_inner_index][4], voxels_with_particle[new_voxel_index][new_voxel_inner_index][5],
                                                        voxels_with_particle[new_voxel_index][new_voxel_inner_index][6]);

            int particle_pyramid_index_new = h_index * observation_pyramid_num_v + v_index;

            int successfully_moved_by_pyramid = 0;
            for(int j=0; j<SAFE_PARTICLE_NUM_PYRAMID; j++){
                if(pyramids_in_fov[particle_pyramid_index_new][j][0]==0){
                    pyramids_in_fov[particle_pyramid_index_new][j][0] |= O_MAKE_VALID;
                    pyramids_in_fov[particle_pyramid_index_new][j][1] = new_voxel_index;
                    pyramids_in_fov[particle_pyramid_index_new][j][2] = new_voxel_inner_index;
                    successfully_moved_by_pyramid = 1;
                    break;
                }
            }

            if(!successfully_moved_by_pyramid){  /// let the particle vanish
                voxels_with_particle[new_voxel_index][new_voxel_inner_index][0] = 0.f; ///vanish
                return -2;
            }

            /// Add Gaussian randoms to velocities of particles inside FOV
            if(fabs(voxels_with_particle[new_voxel_index][new_voxel_inner_index][1]*voxels_with_particle[new_voxel_index][new_voxel_inner_index][2]*voxels_with_particle[new_voxel_index][new_voxel_inner_index][3]) < 1e-6){
                // keep small, for static obstacles
//                cout << "keeped"<<endl;
            }else{
                voxels_with_particle[new_voxel_index][new_voxel_inner_index][1] += getVelocityGaussianZeroCenter();  //vx
                voxels_with_particle[new_voxel_index][new_voxel_inner_index][2] += getVelocityGaussianZeroCenter();  //vy
                voxels_with_particle[new_voxel_index][new_voxel_inner_index][3] = 0.f; //+= getVelocityGaussianZeroCenter();  //vz
            }

        } //else we don't need to consider pyramids

        return 1;
    }


    static void removeParticle(float *ori_particle_flag_ptr){
        *ori_particle_flag_ptr = 0.f;
    }


    static float standardNormalPDF(float value)
    {
        float fx = (1.f/(sqrtf(2.f*M_PI_2f32)))*expf(-powf(value,2)/(2));
        return fx;
    }

    static void calculateNormalPDFBuffer(){
        for(int i=0; i<20000;++i){
            standard_gaussian_pdf[i] = standardNormalPDF((float) (i - 10000) * 0.001f); // range[-10, 10]; 10 sigma
        }
    }

    static float queryNormalPDF(float &x, float &mu, float &sigma)
    {
        float corrected_x = (x-mu)/sigma;
        if(corrected_x>9.9f) corrected_x=9.9f;
        else if(corrected_x<-9.9f) corrected_x=-9.9f;

        return standard_gaussian_pdf[(int)(corrected_x*1000+10000)];
    }

    static void rotateVectorByQuaternion(const float *ori_vector, const float *quaternion, float *rotated_vector)
    {
        //Lazy. Use Eigen directly
        Eigen::Quaternionf ori_vector_quaternion, vector_quaternion;
        ori_vector_quaternion.w() = 0;
        ori_vector_quaternion.x() = *ori_vector;
        ori_vector_quaternion.y() = *(ori_vector+1);
        ori_vector_quaternion.z() = *(ori_vector+2);

        Eigen::Quaternionf att;
        att.w() = *quaternion;
        att.x() = *(quaternion+1);
        att.y() = *(quaternion+2);
        att.z() = *(quaternion+3);

        vector_quaternion = att * ori_vector_quaternion * att.inverse();
        *rotated_vector = vector_quaternion.x();
        *(rotated_vector+1) = vector_quaternion.y();
        *(rotated_vector+2) = vector_quaternion.z();
    }

    static float vectorMultiply(float &x1, float &y1, float &z1, float &x2, float &y2, float &z2){
        return x1*x2 + y1*y2 + z1*z2;
    }

//    static void vectorCOut(float *a){
//        cout<<"("<< *a <<","<< *(a+1)<<","<< *(a+2)<<") ";
//    }

    int ifInPyramidsArea(float &x, float &y, float &z)
    {
//        vectorCOut(&pyramid_BPnorm_params_v[observation_pyramid_num_v][0]);

        if(vectorMultiply(x,y,z, pyramid_BPnorm_params_h[0][0], pyramid_BPnorm_params_h[0][1], pyramid_BPnorm_params_h[0][2]) >= 0.f
          && vectorMultiply(x,y,z, pyramid_BPnorm_params_h[observation_pyramid_num_h][0], pyramid_BPnorm_params_h[observation_pyramid_num_h][1], pyramid_BPnorm_params_h[observation_pyramid_num_h][2]) <= 0.f
          && vectorMultiply(x,y,z, pyramid_BPnorm_params_v[0][0], pyramid_BPnorm_params_v[0][1], pyramid_BPnorm_params_v[0][2]) <= 0.f
          && vectorMultiply(x,y,z, pyramid_BPnorm_params_v[observation_pyramid_num_v][0], pyramid_BPnorm_params_v[observation_pyramid_num_v][1], pyramid_BPnorm_params_v[observation_pyramid_num_v][2]) >= 0.f){
            return 1;
        }else{
            return 0;
        }
    }

    int findPointPyramidHorizontalIndex(float &x, float &y, float &z){  /// The point should already be inside of Pyramids Area
        float last_dot_multiply = 1.f; // for horizontal direction, if the point is inside of Pyramids Area. The symbol of the first dot multiplication should be positive
        for(int i=0; i< observation_pyramid_num_h; i++){
            float this_dot_multiply = vectorMultiply(x, y, z, pyramid_BPnorm_params_h[i+1][0], pyramid_BPnorm_params_h[i+1][1], pyramid_BPnorm_params_h[i+1][2]);
            if(last_dot_multiply * this_dot_multiply <= 0.f){
                return i;
            }
            last_dot_multiply = this_dot_multiply;
        }

        cout << "!!!!!! Please use Function ifInPyramidsArea() to filter the points first before using findPointPyramidHorizontalIndex()" <<endl;
        return -1; // This should not happen if the function is used properly
    }

    int findPointPyramidVerticalIndex(float &x, float &y, float &z){  /// The point should already be inside of Pyramids Area
        float last_dot_multiply = -1.f; // for vertical direction, if the point is inside of Pyramids Area. The symbol of the first dot multiplication should be negative
        for(int j=0; j< observation_pyramid_num_v; j++){
            float this_dot_multiply = vectorMultiply(x, y, z, pyramid_BPnorm_params_v[j+1][0], pyramid_BPnorm_params_v[j+1][1], pyramid_BPnorm_params_v[j+1][2]);
            if(last_dot_multiply * this_dot_multiply <= 0.f){
                return j;
            }
            last_dot_multiply = this_dot_multiply;
        }

        cout << "!!!!!! Please use Function ifInPyramidsArea() to filter the points first before using findPyramidVerticalIndex()" <<endl;
        return -1; // This should not happen if the function is used properly
    }

    static float clusterDistance(ClusterFeature &c1, ClusterFeature &c2){
        float square_distance = (c1.center_x - c2.center_x)*(c1.center_x - c2.center_x) +
                                (c1.center_y - c2.center_y)*(c1.center_y - c2.center_y) +
                                (c1.center_z - c2.center_z)*(c1.center_z - c2.center_z);
        return sqrtf(square_distance);
    }


    static void velocityEstimationThread()
    {
        if( cloud_in_current_view_rotated->points.empty()) return;

        input_cloud_with_velocity->clear();

        /// Remove ground and transform data
        pcl::PointCloud<pcl::PointXYZ>::Ptr static_points(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_points(new pcl::PointCloud<pcl::PointXYZ>());

        for(auto &p : cloud_in_current_view_rotated->points){
            pcl::PointXYZ transformed_p;
            transformed_p.x = p.x + current_position[0];
            transformed_p.y = p.y + current_position[1];
            transformed_p.z = p.z + current_position[2];

            if(transformed_p.z > voxel_filtered_resolution){
                non_ground_points->points.push_back(transformed_p);
            }else{
                static_points->points.push_back(transformed_p);
            }
        }

        /// Cluster
        static std::vector<ClusterFeature> clusters_feature_vector_dynamic_last;
        std::vector<ClusterFeature> clusters_feature_vector_dynamic;
        std::vector<pcl::PointIndices> cluster_indices;
        vector<bool> cluster_possibly_dynamic;

        if(!non_ground_points->empty()){
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (non_ground_points);

            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (2*voxel_filtered_resolution);
            ec.setMinClusterSize (5);
            ec.setMaxClusterSize (10000);

            ec.setSearchMethod (tree);
            ec.setInputCloud (non_ground_points);
            ec.extract (cluster_indices);

            for(const auto & cluster_indice : cluster_indices)
            {
                ClusterFeature cluster_this;
                cluster_this.intensity = generateRandomFloat(0.1f, 1.f); //For visualization

                for (int indice : cluster_indice.indices){
                    cluster_this.center_x += (*non_ground_points)[indice].x; //sum
                    cluster_this.center_y += (*non_ground_points)[indice].y;
                    cluster_this.center_z += (*non_ground_points)[indice].z;
                    ++ cluster_this.point_num;
                }

                // average
                cluster_this.center_x /= (float)cluster_this.point_num;
                cluster_this.center_y /= (float)cluster_this.point_num;
                cluster_this.center_z /= (float)cluster_this.point_num;

                if(cluster_indice.indices.size() > 150 || cluster_this.center_z > 1.4){ //filter static points  //400, 1.5
                    // Static
                    for (int indice : cluster_indice.indices){
                        static_points->push_back((*non_ground_points)[indice]);
                    }
                    cluster_possibly_dynamic.push_back(false);
                }else{
                    // Possibly dynamic
                    clusters_feature_vector_dynamic.push_back(cluster_this);
                    cluster_possibly_dynamic.push_back(true);
                }
            }

            static float distance_gate = 1.5f;
            static int point_num_gate = 100;
            static float maximum_velocity = 5.f;

            /// Move last feature vector d and match by KM algorithm
            if(!clusters_feature_vector_dynamic_last.empty() && !clusters_feature_vector_dynamic.empty()){
                if(delt_t_from_last_observation > 0.00001 && delt_t_from_last_observation < 10.0){
                    Matrix<float> matrix_cost(clusters_feature_vector_dynamic.size(), clusters_feature_vector_dynamic_last.size()); //This is a Matrix defined in munkres.h
                    Matrix<float> matrix_gate(clusters_feature_vector_dynamic.size(), clusters_feature_vector_dynamic_last.size());

                    for(int row=0; row < clusters_feature_vector_dynamic.size(); ++row)
                    {
                        for(int col=0; col < clusters_feature_vector_dynamic_last.size(); ++col){
                            float cluster_distance_this = clusterDistance(clusters_feature_vector_dynamic[row], clusters_feature_vector_dynamic_last[col]);
                            if(abs(clusters_feature_vector_dynamic[row].point_num - clusters_feature_vector_dynamic_last[col].point_num) > point_num_gate
                               || cluster_distance_this >= distance_gate){
                                matrix_gate(row, col) = 0.f;
                                matrix_cost(row, col) = distance_gate * 5000.f;
                            }else{
                                matrix_gate(row, col) = 1.f;
                                matrix_cost(row, col) = cluster_distance_this / distance_gate * 1000.f;
                            }
                        }
                    }

                    Munkres<float> munkres_solver;
                    munkres_solver.solve(matrix_cost);

                    for(int row=0; row < clusters_feature_vector_dynamic.size(); ++row)
                    {
                        for(int col=0; col < clusters_feature_vector_dynamic_last.size(); ++col)
                        {
                            if(matrix_cost(row, col) == 0.f && matrix_gate(row, col) > 0.01f){ // Found a match
                                clusters_feature_vector_dynamic[row].match_cluster_seq = col;
                                clusters_feature_vector_dynamic[row].vx = (clusters_feature_vector_dynamic[row].center_x - clusters_feature_vector_dynamic_last[col].center_x) / delt_t_from_last_observation;
                                clusters_feature_vector_dynamic[row].vy = (clusters_feature_vector_dynamic[row].center_y - clusters_feature_vector_dynamic_last[col].center_y) / delt_t_from_last_observation;
                                clusters_feature_vector_dynamic[row].vz = (clusters_feature_vector_dynamic[row].center_z - clusters_feature_vector_dynamic_last[col].center_z) / delt_t_from_last_observation;
//                        cout << "v=("<<clusters_feature_vector_dynamic[row].vx<<", " << clusters_feature_vector_dynamic[row].vy <<", "<<clusters_feature_vector_dynamic[row].vz << ")" << endl;
                                clusters_feature_vector_dynamic[row].v = sqrtf(clusters_feature_vector_dynamic[row].vx * clusters_feature_vector_dynamic[row].vx + clusters_feature_vector_dynamic[row].vy * clusters_feature_vector_dynamic[row].vy + clusters_feature_vector_dynamic[row].vz * clusters_feature_vector_dynamic[row].vz);
                                clusters_feature_vector_dynamic[row].intensity = clusters_feature_vector_dynamic_last[col].intensity; //for visualization

                                if(clusters_feature_vector_dynamic[row].v > maximum_velocity){
                                    clusters_feature_vector_dynamic[row].v = 0.f;
                                    clusters_feature_vector_dynamic[row].vx = clusters_feature_vector_dynamic[row].vy = clusters_feature_vector_dynamic[row].vz = 0.f;
                                }

                                break;
                            }
                            /// If no match is found. The cluster velocity is given by struct initialization (v=-1000).
                        }
                    }
                }
            }

            /// Velocity Allocation to Points
            // Use normal to store velocity
            int cluster_indice_seq = 0;
            int cluster_dynamic_vector_seq = 0;
            for(const auto & cluster_indice : cluster_indices) {
                if(cluster_possibly_dynamic[cluster_indice_seq]){
                    for (int indice : cluster_indice.indices) {
                        pcl::PointXYZINormal p;
                        p.x = (*non_ground_points)[indice].x;
                        p.y = (*non_ground_points)[indice].y;
                        p.z = (*non_ground_points)[indice].z;
                        p.normal_x = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].vx;  // Use color to store velocity
                        p.normal_y = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].vy;
                        p.normal_z = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].vz;
                        p.intensity = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].intensity; // For visualization. // clusters_feature_vector_dynamic[cluster_indice_seq].v / maximum_velocity;
                        input_cloud_with_velocity->push_back(p);
                    }
                    ++ cluster_dynamic_vector_seq;
                }

                ++ cluster_indice_seq;
            }

        }


        for(auto &static_point : static_points->points)
        {
            pcl::PointXYZINormal p;
            p.x = static_point.x;
            p.y = static_point.y;
            p.z = static_point.z;
            p.normal_x = 0.f;
            p.normal_y = 0.f;
            p.normal_z = 0.f;
            p.intensity = 0.f;
            input_cloud_with_velocity->push_back(p);
        }

        clusters_feature_vector_dynamic_last = clusters_feature_vector_dynamic;
        cout << "Velocity estimation done" << endl;
    }




    /*** For test ***/
public:
    static float generateRandomFloat(float min, float max){
        return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
    }


    void getVoxelPositionFromIndexPublic(const int &index, float &px, float &py, float &pz) const{
        static const int z_change_storage_taken = voxel_num_y*voxel_num_x;
        static const int y_change_storage_taken = voxel_num_x;

        int z_index = index / z_change_storage_taken;
        int yx_left_indexes = index - z_index * z_change_storage_taken;
        int y_index = yx_left_indexes / y_change_storage_taken;
        int x_index = yx_left_indexes - y_index * y_change_storage_taken;

        static const float correction_x = -map_length_x_half + voxel_resolution*0.5f;
        static const float correction_y = -map_length_y_half + voxel_resolution*0.5f;
        static const float correction_z = -map_length_z_half + voxel_resolution*0.5f;

        px = (float)x_index * voxel_resolution + correction_x;
        py = (float)y_index * voxel_resolution + correction_y;
        pz = (float)z_index * voxel_resolution + correction_z;
    }

    int getPointVoxelsIndexPublic(const float &px, const float &py, const float &pz, int & index){
        if(ifParticleIsOut(px, py, pz)) {return 0;}
        auto x = (int)((px + map_length_x_half) / voxel_resolution);
        auto y = (int)((py + map_length_y_half) / voxel_resolution);
        auto z = (int)((pz + map_length_z_half) / voxel_resolution);
        index = z*voxel_num_y*voxel_num_x + y*voxel_num_x + x;
        if(index<0 || index>=VOXEL_NUM){
            return 0;
        }
        return 1;
    }

};



