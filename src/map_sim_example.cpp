//
// Created by clarence on 2021/4/12.
//

#include "ros/ros.h"
#include "dsp-nonegaussian-dst-new-multiple-neighbors.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Pose.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TwistStamped.h>
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseStamped.h"
#include <queue>
#include "std_msgs/Float64.h"

DSPMap my_map;

queue<double> pose_att_time_queue;
queue<Eigen::Vector3d> uav_position_global_queue;
queue<Eigen::Quaternionf> uav_att_global_queue;

const unsigned int MAX_POINT_NUM  = 5000;
float point_clouds[MAX_POINT_NUM*3];

float x_min = -MAP_LENGTH_VOXEL_NUM * VOXEL_RESOLUTION / 2;
float x_max = MAP_LENGTH_VOXEL_NUM * VOXEL_RESOLUTION / 2;
float y_min = -MAP_WIDTH_VOXEL_NUM * VOXEL_RESOLUTION / 2;
float y_max = MAP_WIDTH_VOXEL_NUM * VOXEL_RESOLUTION / 2;
float z_min = -MAP_HEIGHT_VOXEL_NUM * VOXEL_RESOLUTION / 2;
float z_max = MAP_HEIGHT_VOXEL_NUM * VOXEL_RESOLUTION / 2;

ros::Publisher cloud_pub, map_center_pub, gazebo_model_states_pub, current_velocity_pub, single_object_velocity_pub, single_object_velocity_truth_pub;
ros::Publisher future_status_pub, current_marker_pub, fov_pub, update_time_pub, cluster_status_pub;
gazebo_msgs::ModelStates ground_truth_model_states;

Eigen::Vector3d uav_position_global;
Eigen::Quaternionf uav_att_global;

float res = 0.1;  // Smaller res will get better tracking result but is slow.
bool state_locked = false;


void actor_publish(const vector<Eigen::Vector3d> &actors, int id, float r, float g, float b, float width, int publish_num)
{
    if(actors.empty()) return;

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "actors";

    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 1.7;
    marker.color.a = 0.6;
    marker.color.r = 0.3;
    marker.color.g = 0.3;
    marker.color.b = 0.9;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    for(int i=0; i<actors.size(); ++i)
    {
        marker.id = i;
        marker.pose.position.x = actors[i].x();
        marker.pose.position.y = actors[i].y();
        marker.pose.position.z = actors[i].z();
        marker_array.markers.push_back(marker);
    }

    current_marker_pub.publish(marker_array);
}

static void rotateVectorByQuaternion(geometry_msgs::Point &vector, Eigen::Quaternionf att)
{
    //Lazy. Use Eigen directly
    Eigen::Quaternionf ori_vector_quaternion, vector_quaternion;
    ori_vector_quaternion.w() = 0;
    ori_vector_quaternion.x() = vector.x;
    ori_vector_quaternion.y() = vector.y;
    ori_vector_quaternion.z() = vector.z;

    vector_quaternion = att * ori_vector_quaternion * att.inverse();
    vector.x = vector_quaternion.x();
    vector.y = vector_quaternion.y();
    vector.z = vector_quaternion.z();
}


void showFOV(Eigen::Vector3d &position, Eigen::Quaternionf &att, double angle_h, double angle_v, double length){
    geometry_msgs::Point p_cam;
    p_cam.x = 0;
    p_cam.y = 0;
    p_cam.z = 0;

    geometry_msgs::Point p1, p2, p3, p4;
    p1.x = length;
    p1.y = length * tan(angle_h/2);
    p1.z = length * tan(angle_v/2);
    rotateVectorByQuaternion(p1, att);

    p2.x = -length;
    p2.y = length * tan(angle_h/2);
    p2.z = length * tan(angle_v/2);
    rotateVectorByQuaternion(p2, att);

    p3.x = length;
    p3.y = length * tan(angle_h/2);
    p3.z = -length * tan(angle_v/2);
    rotateVectorByQuaternion(p3, att);

    p4.x = -length;
    p4.y = length * tan(angle_h/2);
    p4.z = -length * tan(angle_v/2);
    rotateVectorByQuaternion(p4, att);

    visualization_msgs::Marker fov;
    fov.header.frame_id = "map";
    fov.header.stamp = ros::Time::now();
    fov.action = visualization_msgs::Marker::ADD;
    fov.ns = "lines_and_points";
    fov.id = 999;
    fov.type = 4;

    fov.scale.x = 0.1;
    fov.scale.y = 0.1;
    fov.scale.z = 0.1;

    fov.color.r = 0.8;
    fov.color.g = 0.5;
    fov.color.b = 0.5;
    fov.color.a = 0.8;
    fov.lifetime = ros::Duration(0);

    fov.points.push_back(p1);
    fov.points.push_back(p2);
    fov.points.push_back(p_cam);
    fov.points.push_back(p4);
    fov.points.push_back(p3);
    fov.points.push_back(p_cam);
    fov.points.push_back(p1);
    fov.points.push_back(p3);
    fov.points.push_back(p4);
    fov.points.push_back(p2);
    fov_pub.publish(fov);
}

int inRange(float &low, float &high, float &x)
{
    if(x > low && x < high){
        return 1;
    }else{
        return 0;
    }
}

void colorAssign(int &r, int &g, int &b, float v, float value_min=0.f, float value_max=1.f, int reverse_color=0)
{
    v = std::max(v, value_min);
    v = std::min(v, value_max);

    float v_range = value_max - value_min;
    int value = floor((v - value_min) / v_range * 240); // Mapping 0~1.0 to 0~240
    value = std::min(value, 240);

    if(reverse_color){
        value = 240 - value;
    }

    int section = value / 60;
    float float_key = (value % 60) / (float)60 * 255;
    int key = floor(float_key);
    int nkey = 255 - key;

    switch(section) {
        case 0: // G increase
            r = 255;
            g = key;
            b = 0;
            break;
        case 1: // R decrease
            r = nkey;
            g = 255;
            b = 0;
            break;
        case 2: // B increase
            r = 0;
            g = 255;
            b = key;
            break;
        case 3: // G decrease
            r = 0;
            g = nkey;
            b = 255;
            break;
        case 4:
            r = 0;
            g = 0;
            b = 255;
            break;
        default: // White
            r = 255;
            g = 255;
            b = 255;
    }

}


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    /// Simple synchronizer
    Eigen::Vector3d uav_position = uav_position_global;
    Eigen::Quaternionf uav_att = uav_att_global;

    static Eigen::Quaternionf quad_last_popped(-10.f, -10.f, -10.f, -10.f);
    static Eigen::Vector3d position_last_popped(-10000.f, -10000.f, -10000.f);
    static double last_popped_time = 0.0;

    ros::Rate loop_rate(500);
    while(state_locked){
        loop_rate.sleep();
        ros::spinOnce();
    }
    state_locked = true;

    while(!pose_att_time_queue.empty()){   //Synchronize pose by queue
        double time_stamp_pose = pose_att_time_queue.front();
        if(time_stamp_pose >= cloud->header.stamp.toSec()){
            uav_att = uav_att_global_queue.front();
            uav_position = uav_position_global_queue.front();

            // linear interpolation
            if(quad_last_popped.x() >= -1.f){
                double time_interval_from_last_time = time_stamp_pose - last_popped_time;
                double time_interval_cloud = cloud->header.stamp.toSec() - last_popped_time;
                double factor = time_interval_cloud / time_interval_from_last_time;
                uav_att = quad_last_popped.slerp(factor, uav_att);
                uav_position = position_last_popped * (1.0 - factor) + uav_position*factor;
            }

            ROS_INFO_THROTTLE(3.0, "cloud mismatch time = %lf", cloud->header.stamp.toSec() - time_stamp_pose);

            break;
        }

        quad_last_popped = uav_att_global_queue.front();
        position_last_popped = uav_position_global_queue.front();
        last_popped_time = time_stamp_pose;

        pose_att_time_queue.pop();
        uav_att_global_queue.pop();
        uav_position_global_queue.pop();
    }
    state_locked = false;


    /// Point cloud process
    double this_time = cloud->header.stamp.toSec();

    // convert cloud to pcl form
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *cloud_in);

    // down-sample for all
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(res, res, res);
    sor.filter(*cloud_filtered);

    int useful_point_num = 0;
    for(int i=0; i<cloud_filtered->width; i++){
        float x = cloud_filtered->points.at(i).z;
        float y = -cloud_filtered->points.at(i).x;
        float z = -cloud_filtered->points.at(i).y;

        if(inRange(x_min, x_max, x) && inRange(y_min, y_max, y) && inRange(z_min, z_max, z))
        {
            point_clouds[useful_point_num*3] = x;
            point_clouds[useful_point_num*3+1] = y;
            point_clouds[useful_point_num*3+2] = z;
            ++ useful_point_num;

            if(useful_point_num >= MAX_POINT_NUM){
                break;
            }
        }
    }

    /// Update
    clock_t start1, finish1;
    start1 = clock();

    std::cout << "uav_position="<<uav_position.x() <<", "<<uav_position.y()<<", "<<uav_position.z()<<endl;

    if(!my_map.update(useful_point_num, 3, point_clouds,
                  uav_position.x(), uav_position.y(), uav_position.z(), this_time,
                  uav_att.w(), uav_att.x(), uav_att.y(), uav_att.z())){
        return;
    }

    /// Display update time
    finish1 = clock();
    double duration1 = (double)(finish1 - start1) / CLOCKS_PER_SEC;
    printf( "****** Map update time %f seconds\n", duration1);

    static double total_time = 0.0;
    static int update_times = 0;

    total_time += duration1;
    update_times ++;
    printf( "****** Map avg time %f seconds\n \n", total_time / update_times);


    /// Get occupancy status
    clock_t start2, finish2;   start2 = clock();

    int occupied_num=0;
    pcl::PointCloud<pcl::PointXYZ> cloud_to_publish;
    sensor_msgs::PointCloud2 cloud_to_pub_transformed;
    static float future_status[VOXEL_NUM][PREDICTION_TIMES];

    my_map.getOccupancyMapWithFutureStatus(occupied_num, cloud_to_publish, &future_status[0][0], 0.33); //0.25

    /// Publish Point cloud and center position
    pcl::toROSMsg(cloud_to_publish, cloud_to_pub_transformed);
    cloud_to_pub_transformed.header.frame_id = "map";
    cloud_to_pub_transformed.header.stamp = cloud->header.stamp;
    cloud_pub.publish(cloud_to_pub_transformed);

    geometry_msgs::PoseStamped map_pose;
    map_pose.header.stamp = cloud_to_pub_transformed.header.stamp;
    map_pose.pose.position.x = uav_position.x();
    map_pose.pose.position.y = uav_position.y();
    map_pose.pose.position.z = uav_position.z();
    map_pose.pose.orientation.x = uav_att.x();
    map_pose.pose.orientation.y = uav_att.y();
    map_pose.pose.orientation.z = uav_att.z();
    map_pose.pose.orientation.w = uav_att.w();
    map_center_pub.publish(map_pose);


    /// Publish future status of one layer
    pcl::PointCloud<pcl::PointXYZRGB> future_status_cloud;
    static const int z_index_to_show = MAP_HEIGHT_VOXEL_NUM / 2 - 1; ///Layer
    for(int j=0; j<MAP_WIDTH_VOXEL_NUM; ++j){
        for(int i=0; i<MAP_LENGTH_VOXEL_NUM; ++i){
            int index_this = z_index_to_show*MAP_WIDTH_VOXEL_NUM*MAP_LENGTH_VOXEL_NUM + j*MAP_WIDTH_VOXEL_NUM + i;

            for(int n=0; n<PREDICTION_TIMES; ++n){
                pcl::PointXYZRGB p_this;
                float x_offset = (float)n * 12.f; //Used to show prediction at different times in one map

                my_map.getVoxelPositionFromIndexPublic(index_this, p_this.x, p_this.y, p_this.z);
                p_this.x += x_offset;

                float weight_this = future_status[index_this][n];
                int r, g, b;
                colorAssign(r, g, b, weight_this, 0.f, 0.1f, 1);
                p_this.r = r;
                p_this.g = g;
                p_this.b = b;
                future_status_cloud.push_back(p_this);
            }
        }
    }

    sensor_msgs::PointCloud2 cloud_future_transformed;
    pcl::toROSMsg(future_status_cloud, cloud_future_transformed);
    cloud_future_transformed.header.frame_id = "map";
    cloud_future_transformed.header.stamp = cloud->header.stamp;
    future_status_pub.publish(cloud_future_transformed);

    finish2 = clock();

    double duration2 = (double)(finish2 - start2) / CLOCKS_PER_SEC;
    printf( "****** Map publish time %f seconds\n \n", duration2);

    /// Visualize initial velocity estimation clusters
    pcl::PointCloud<pcl::PointXYZINormal> cluster_cloud;
    my_map.getKMClusterResult(cluster_cloud);

    sensor_msgs::PointCloud2 cluster_cloud_ros;
    pcl::toROSMsg(cluster_cloud, cluster_cloud_ros);

    cluster_cloud_ros.header.frame_id = "map";
    cluster_cloud_ros.header.stamp = cloud->header.stamp;
    cluster_status_pub.publish(cluster_cloud_ros);


    /// For evaluation tools
    std_msgs::Float64 update_time;
    update_time.data = duration1 + duration2;
    update_time_pub.publish(update_time);

}

static void split(const string& s, vector<string>& tokens, const string& delimiters = " ")
{
    string::size_type lastPos = s.find_first_not_of(delimiters, 0);
    string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (string::npos != pos || string::npos != lastPos) {
        tokens.push_back(s.substr(lastPos, pos - lastPos));
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
}


void simObjectStateCallback(const gazebo_msgs::ModelStates &msg)
{
    ground_truth_model_states = msg;

    vector<Eigen::Vector3d> actor_visualization_points;

    for(int i=0; i<msg.name.size(); ++i)
    {
        vector<string> name_splited;
        split(msg.name[i], name_splited, "_");
        if(name_splited[0] == "actor"){
            Eigen::Vector3d p;
            p.x() = msg.pose[i].position.x - uav_position_global.x();
            p.y() = msg.pose[i].position.y - uav_position_global.y();
            p.z() = msg.pose[i].position.z - uav_position_global.z();
            actor_visualization_points.push_back(p);
        }
    }
    actor_publish(actor_visualization_points, 6, 1.f, 0.f, 0.f, 0.8, -1);
    gazebo_model_states_pub.publish(ground_truth_model_states);
}

void simPoseCallback(const geometry_msgs::PoseStamped &msg)
{
    if(!state_locked)
    {
        state_locked = true;
        uav_position_global.x() = msg.pose.position.x;
        uav_position_global.y() = msg.pose.position.y;
        uav_position_global.z() = msg.pose.position.z;

        uav_att_global.x() = msg.pose.orientation.x;
        uav_att_global.y() = msg.pose.orientation.y;
        uav_att_global.z() = msg.pose.orientation.z;
        uav_att_global.w() = msg.pose.orientation.w;

        uav_position_global_queue.push(uav_position_global);
        uav_att_global_queue.push(uav_att_global);
        pose_att_time_queue.push(msg.header.stamp.toSec());
        ROS_INFO("Pose updated");
    }

    state_locked = false;

    Eigen::Quaternionf axis; //= quad * q1 * quad.inverse();
    axis.w() = cos(-M_PI/4.0);
    axis.x() = 0.0;
    axis.y() = 0.0;
    axis.z() = sin(-M_PI/4.0);
    Eigen::Quaternionf rotated_att = uav_att_global * axis;

    showFOV(uav_position_global, rotated_att, 90.0 / 180.0 * M_PI, 54.0 / 180.0 * M_PI , 5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_sim_example_with_cluster");
    ros::NodeHandle n;

    my_map.setPredictionVariance(0.05, 0.05);  //0.1, 0.1
    my_map.setObservationStdDev(0.1); //0.02, 0.2(new)
//    my_map.setParticleRecordFlag(1, 19.2);
    my_map.setNewBornParticleNumberofEachPoint(20); //30
    my_map.setNewBornParticleWeight(0.0001); //0.01
    DSPMap::setOriginalVoxelFilterResolution(res);

    ros::Subscriber object_states_sub = n.subscribe("/gazebo/model_states", 1, simObjectStateCallback);
    ros::Subscriber point_cloud_sub = n.subscribe("/d400/depth/color/points", 1, cloudCallback);
    ros::Subscriber pose_sub = n.subscribe("/mavros/local_position/pose", 1, simPoseCallback);

    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/my_map/cloud_ob", 1, true);
    map_center_pub = n.advertise<geometry_msgs::PoseStamped>("/my_map/map_center", 1, true);
    gazebo_model_states_pub = n.advertise<gazebo_msgs::ModelStates>("/my_map/model_states", 1, true);

    future_status_pub = n.advertise<sensor_msgs::PointCloud2>("/my_map/future_status", 1, true);
    cluster_status_pub = n.advertise<sensor_msgs::PointCloud2>("/my_map/cluster_status", 1, true);

    current_velocity_pub = n.advertise<visualization_msgs::MarkerArray>("/my_map/velocity_marker", 1);
    single_object_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/my_map/single_object_velocity", 1);
    single_object_velocity_truth_pub = n.advertise<geometry_msgs::TwistStamped>("/my_map/single_object_velocity_ground_truth", 1);
    current_marker_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker", 1);
    fov_pub = n.advertise<visualization_msgs::Marker>("/visualization_fov", 1);

    update_time_pub = n.advertise<std_msgs::Float64>("/map_update_time", 1);

    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}