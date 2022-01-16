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

/** Parameters for the map **/
#define MAP_LENGTH_VOXEL_NUM 66
#define MAP_WIDTH_VOXEL_NUM 66
#define MAP_HEIGHT_VOXEL_NUM 41
#define VOXEL_RESOLUTION 0.15
#define ANGLE_RESOLUTION 1
#define MAX_PARTICLE_NUM_VOXEL 25
#define LIMIT_MOVEMENT_IN_XY_PLANE 1
#define PYRAMID_NEIGHBOR_N 1


#define PREDICTION_TIMES 6
static const float prediction_future_time[PREDICTION_TIMES] = {0.05f, 0.2f, 0.5f, 1.f, 1.5f, 2.f}; //unit: second

const int half_fov_h = 45;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make half_fov_h a smaller value than the real FOV angle
const int half_fov_v = 27;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make half_fov_h a smaller value than the real FOV angle

/** END **/

static const int observation_pyramid_num_h = (int)half_fov_h * 2 / ANGLE_RESOLUTION;
static const int observation_pyramid_num_v = (int)half_fov_v * 2 / ANGLE_RESOLUTION;
static const int observation_pyramid_num = observation_pyramid_num_h * observation_pyramid_num_v;

static const int VOXEL_NUM = MAP_LENGTH_VOXEL_NUM*MAP_WIDTH_VOXEL_NUM*MAP_HEIGHT_VOXEL_NUM;
static const int PYRAMID_NUM = 360*180/ANGLE_RESOLUTION/ANGLE_RESOLUTION;
static const int SAFE_PARTICLE_NUM = VOXEL_NUM * MAX_PARTICLE_NUM_VOXEL + 1e5;
static const int SAFE_PARTICLE_NUM_VOXEL = MAX_PARTICLE_NUM_VOXEL * 2;
static const int SAFE_PARTICLE_NUM_PYRAMID = SAFE_PARTICLE_NUM/PYRAMID_NUM * 2;

static const int observation_max_points_num_one_pyramid = 100;

#define GAUSSIAN_RANDOMS_NUM 10000000

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

/// Container for voxels h particles
static float voxels_with_particle[VOXEL_NUM][SAFE_PARTICLE_NUM_VOXEL][9];

static const int voxels_objects_number_dimension = 4 + PREDICTION_TIMES;
static float voxels_objects_number[VOXEL_NUM][voxels_objects_number_dimension];

/// Container for pyramids
static int pyramids_in_fov[observation_pyramid_num][SAFE_PARTICLE_NUM_PYRAMID][3];

static const int pyramid_neighbors_num = (2 * PYRAMID_NEIGHBOR_N + 1) * (2 * PYRAMID_NEIGHBOR_N + 1);
static int observation_pyramid_neighbors[observation_pyramid_num][pyramid_neighbors_num+1]{};

/// Variables for velocity estimation
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_current_view_rotated(new pcl::PointCloud<pcl::PointXYZ>());
static float current_position[3] = {0.f, 0.f, 0.f};
static float voxel_filtered_resolution = 0.15;
static float delt_t_from_last_observation = 0.f;
pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud_with_velocity(new pcl::PointCloud<pcl::PointXYZINormal>());

/** Storage for Gaussian randoms and Gaussian PDF**/
static float p_gaussian_randoms[GAUSSIAN_RANDOMS_NUM];
static float v_gaussian_randoms[GAUSSIAN_RANDOMS_NUM];
static float standard_gaussian_pdf[20000];

class DSPMap{
public:

    DSPMap(int init_particle_num = 0, float init_weight=0.01f)
            : voxel_num_x(MAP_LENGTH_VOXEL_NUM),
              voxel_num_y(MAP_WIDTH_VOXEL_NUM),
              voxel_num_z(MAP_HEIGHT_VOXEL_NUM),
              voxel_resolution(VOXEL_RESOLUTION),
              angle_resolution(ANGLE_RESOLUTION),
              max_particle_num_voxel(MAX_PARTICLE_NUM_VOXEL),
              velocity_gaussian_random_seq(0),
              position_gaussian_random_seq(0),
              position_prediction_stddev(0.2f),
              velocity_prediction_stddev(0.1f),
              sigma_ob(0.2f),
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


    void setPredictionVariance(float p_stddev, float v_stddev){
        position_prediction_stddev = p_stddev;
        velocity_prediction_stddev = v_stddev;
        // regenerate randoms
        generateGaussianRandomsVectorZeroCenter();
    }

    void setObservationStdDev(float ob_stddev){
        sigma_ob = ob_stddev;
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

    float sigma_ob;

    float P_detection;

    int if_record_particle_csv;
    float record_time;

    /** Variables **/
    int position_gaussian_random_seq;
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

                    voxels_with_particle[v_index][p][4] += delt_t*voxels_with_particle[v_index][p][1] + odom_delt_px;  //px
                    voxels_with_particle[v_index][p][5] += delt_t*voxels_with_particle[v_index][p][2] + odom_delt_py;  //py
                    voxels_with_particle[v_index][p][6] += delt_t*voxels_with_particle[v_index][p][3] + odom_delt_pz;  //pz

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

        if(moves_out_counter > 10000){
            cout <<"!!!!! An error occured! delt_t = " << delt_t <<endl;
            cout << "odom_delt_px = " << odom_delt_px <<" odom_delt_py = " << odom_delt_py << "odom_delt_pz=" << odom_delt_pz<< endl;
        }

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

        for(int i=-PYRAMID_NEIGHBOR_N ; i <= PYRAMID_NEIGHBOR_N; ++i){
            for(int j=-PYRAMID_NEIGHBOR_N; j <= PYRAMID_NEIGHBOR_N; ++j){
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
        std::normal_distribution<double> n1(0, position_prediction_stddev);
        std::normal_distribution<double> n2(0, velocity_prediction_stddev);

        for(int i=0; i<GAUSSIAN_RANDOMS_NUM; i++){
            *(p_gaussian_randoms+i) = n1(random);
            *(v_gaussian_randoms+i) = n2(random);
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


    int ifInPyramidsArea(float &x, float &y, float &z)
    {
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

};



