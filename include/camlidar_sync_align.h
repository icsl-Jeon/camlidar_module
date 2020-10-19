#ifndef _CAMLIDAR_SYNC_ALIGN_H_
#define _CAMLIDAR_SYNC_ALIGN_H_

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <sstream>
#include <fstream> // for lidar pcd files

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/eigen.hpp"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// keyboard input tool
#include "keyinput.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// topic synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// dbscan
#include "dbscan.h"
#include <tf/transform_listener.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType,
                                    std::string frameId, ros::Time t);

using namespace std;
inline string dtos(double x) {
	stringstream s;
	s << setprecision(6) << fixed << x;
	return s.str();
};

inline string itos(double x) {
	stringstream s;
	s << x;
	return s.str();
};

class CamLidarSyncAlign {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    CamLidarSyncAlign(ros::NodeHandle& nh,const string& param_path_,const string& save_path);
    ~CamLidarSyncAlign();
    void saveLidarDataRingTime(const string& file_name);
    void saveSnapshot();
    bool getDataReady(){return this->data_ready_;};

    void pntBBQuery(cv::Rect boundingBox ); // JBS
    void pntPixelQuery(const vector<cv::Point>& queryPntSet); // JBS
    void publish() ; // publish routine
    cv::Mat getUndistortedImage() {return img_undistort_;}; // JBS
    bool isReceived = false; // pcl and image all received

private: // ROS related
    string baselink_frame;
    string map_frame;
    ros::NodeHandle nh_;
    ros::Publisher pubBBQueriedPCL;

    ros::Publisher pubBBQueriedPCLProcess;
    ros::Publisher pubVelodyneOriginalTime;
    ros::Publisher pubCaliImage;

    tf::TransformListener tf_l;

    bool isBBQueried = false;
    bool isQueryValid = false; // false if the number of queried points are zero
    bool isPixelQueried = false;
    sensor_msgs::PointCloud2 receivedVelodyne;
    cv::Rect lastBBQuery;

     // subscribers
    message_filters::Subscriber<sensor_msgs::Image> *img_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *lidar_sub;
    message_filters::Synchronizer<MySyncPolicy> *sync_sub;

    // topic names
    string topicname_img_;
    string topicname_lidar_;

    // data container (buffer)
    cv::Mat buf_img_; // Images from mvBlueCOUGAR-X cameras.
    cv::Mat img_undistort_; // image (undistorted, CV_32F (float))
    cv::Mat img_index_; // (CV_16S, -32768~32767) index image (having same size with img_undistort_) storing indexes of warped LiDAR points.
    cv::Mat img_cali_; // calibration turning image

    double buf_time_; // triggered time stamp from Arduino. (seconds)
    pcl::PointCloud<pcl::PointXYZI>::Ptr buf_lidar_; // point clouds (w/ intensity) from Velodyne VLP16 
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_pcl_warped_; // Pointclouds (warped onto the image frame from the LiDAR frame)
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastQueriedPoints; // JBS
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastQueriedPointsProcess; // JBS


    // points w.r.t velodyne
    float* buf_lidar_x;
    float* buf_lidar_y;
    float* buf_lidar_z;

    // Information of transformed point  by Tcl
    float* buf_lidar_x_warped;
    float* buf_lidar_y_warped;
    float* buf_lidar_z_warped;

    float* buf_lidar_u_projected;
    float* buf_lidar_v_projected;
    
    float* buf_lidar_intensity;
    unsigned short* buf_lidar_ring;
    float* buf_lidar_time;

    int n_pts_lidar;
    int n_pts_lidar_warped; // n_pts_lidar_warped < n_pts_lidar

    int current_seq_; // for snapshot saving.
    bool data_ready_;
    string save_dir_;

private: // image undistorter & lidar warping.
    cv::Mat cvK; // intrinsic matrix of the camera.
    cv::Mat cvDistortion; // distortion parameters (five). {k1 k2 p1 p2 k3}
    cv::Mat cvT_cl;

    Eigen::Matrix3d K;
    Eigen::Matrix4d T_cl; // SE(3) transform from a camera frame to a LiDAR frame.
    Eigen::Matrix4d T_bl; // SE(3) transform from a baselink frame to a LiDAR frame (for my drone setting).
    Eigen::Matrix4d T_wb; // SE(3) transform from a world frame to a baselink (for my drone setting).
    Eigen::Vector3f prevTargetClusterCenter;

    int n_cols; // image width
    int n_rows; // image height

    cv::Mat undist_map_x; // undistortion map x (pre-calculated in 'preCalculateUndistortMaps' for speed)
    cv::Mat undist_map_y; // undistortion map y (pre-calculated in 'preCalculateUndistortMaps' for speed)

    float dbscan_eps;
    int dbscan_min_pnts;

    // for debug image
    bool flag_debugimage_;
    string winname_;

// private methods
private: // ROS related
    void callbackImageLidarSync(const sensor_msgs::ImageConstPtr& msg_image, const sensor_msgs::PointCloud2ConstPtr& msg_lidar);

private: // image undistortion & LiDAR warping
    void readCameraLidarParameter(const string& path_dir);
    void preCalculateUndistortMaps();
    void undistortCurrentImage(const cv::Mat& img_source, cv::Mat& img_dst);
    void warpAndProjectLidarPointcloud(); // This function generates "depth image" and "warped lidar pointcloud (R^{3 x N})" 


    // What you can use: "IN THE CALLBACK function"
    // buf_lidar_x_warped, buf_lidar_y_warped, buf_lidar_z_warped, img_undistort_,img_index_;

};


#endif
