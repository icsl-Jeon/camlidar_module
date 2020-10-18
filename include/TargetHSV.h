//
// Created by jbs on 20. 10. 12..
//

#ifndef CAMLIDAR_MODULE_TARGETHSV_H
#define CAMLIDAR_MODULE_TARGETHSV_H

#include <camlidar_sync_align.h>

struct ThresHSV{

    int iLowH; //= 0;
    int iHighH; // = 179;

    int iLowS; //= 0;
    int iHighS; //= 255;

    int iLowV; //= 0;
    int iHighV; //= 255;

    int circle_detect_param1;
    int circle_detect_param2;

};


class TargetHSV{
private:
    ThresHSV target_hsv_thres;
    CamLidarSyncAlign* cl;
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher pubThresMask;
    bool setImshow = true;
    vector<cv::Point> curTargetPixels;

    cv::Mat thresImage;
    cv::Mat undistortImage;
    bool update();
    void thresholding();
    string window_name = "hsv thresholded" ;
    void initTreshWindow();
    void publish();
    bool uploadTargetPixel();

public:
    explicit TargetHSV();
    void run();



};



#endif //CAMLIDAR_MODULE_TARGETHSV_H
