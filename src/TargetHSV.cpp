//
// Created by jbs on 20. 10. 12..
//

#include "TargetHSV.h"

sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t) {
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image& imgMessage = *ptr;
    imgMessage.header.stamp = t;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1; //for endianness detection
    imgMessage.is_bigendian = !(*(char *) &num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);

    if (img.isContinuous())
        memcpy((char*) (&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*) (&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++) {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }
    return ptr;
}


TargetHSV::TargetHSV():nh("~"),it(nh){
    string param_directory;
    nh.param<string>("parameter_directory", param_directory,
            "/home/jbs/test_ws/src/camlidar_module/params/bluefox_vlp16_BS.yaml");

    // Camlidar Sync
    cl = new CamLidarSyncAlign(nh,param_directory,"");


    nh.param("H_max",target_hsv_thres.iHighH,37);
    nh.param("H_min",target_hsv_thres.iLowH,0);
    nh.param("S_max",target_hsv_thres.iHighS,255);
    nh.param("S_min",target_hsv_thres.iLowS,88);
    nh.param("V_max",target_hsv_thres.iHighV,235);
    nh.param("V_min",target_hsv_thres.iLowV,53);
    nh.param("circle_param1",target_hsv_thres.circle_detect_param1,5);
    nh.param("circle_param2",target_hsv_thres.circle_detect_param2,5);

    pubThresMask= it.advertise("undistorted_thres_mask",1);
    initTreshWindow();

}

/**
 * Convert RGB to HSV and perform thresholding
 */
void TargetHSV::thresholding(){

    cv::Mat HSV;
    cvtColor(cl->getUndistortedImage(), HSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    cv::inRange(HSV, cv::Scalar(target_hsv_thres.iLowH, target_hsv_thres.iLowS, target_hsv_thres.iLowV),
            cv::Scalar(target_hsv_thres.iHighH, target_hsv_thres.iHighS, target_hsv_thres.iHighV),
            thresImage); //Threshold the image

    //morphological opening (remove small objects from the foreground)
    cv::erode(thresImage, thresImage, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(target_hsv_thres.circle_detect_param1, target_hsv_thres.circle_detect_param1)));
    cv::dilate(thresImage, thresImage, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(target_hsv_thres.circle_detect_param1, target_hsv_thres.circle_detect_param1)));

    //morphological closing (fill small holes in the foreground)
    cv::dilate(thresImage, thresImage, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(target_hsv_thres.circle_detect_param2, target_hsv_thres.circle_detect_param2)));
    cv::erode(thresImage, thresImage, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(target_hsv_thres.circle_detect_param2, target_hsv_thres.circle_detect_param2)));
//    cout << thresImage << endl;
}

void TargetHSV::initTreshWindow() {
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    //Create trackbars in "Control" window
    cv::createTrackbar("LowH", window_name,  &(target_hsv_thres.iLowH), 255); //Hue (0 - 179)
    cv::createTrackbar("HighH", window_name, &(target_hsv_thres.iHighH), 255);
    cv::createTrackbar("LowS", window_name,  &(target_hsv_thres.iLowS), 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", window_name, &(target_hsv_thres.iHighS), 255);
    cv::createTrackbar("LowV", window_name,  &(target_hsv_thres.iLowV), 255); //Value (0 - 255)
    cv::createTrackbar("HighV", window_name, &(target_hsv_thres.iHighV), 255);
    cv::createTrackbar("circle_param1", window_name,  &(target_hsv_thres.circle_detect_param1), 12); //Value (0 - 255)
    cv::createTrackbar("circle_param2", window_name, &(target_hsv_thres.circle_detect_param2), 12);

}


void TargetHSV::publish() {
    pubThresMask.publish(imageToROSmsg(thresImage,sensor_msgs::image_encodings::BGR8,"bluefox",ros::Time::now()));

}
/**
 * Update the thres hold image
 * @return
 */
bool TargetHSV::update() {

    if (not cl->isReceived){
        ROS_WARN_THROTTLE(2,"Still image and pcl not received.");
        return false;
    }else{
        ROS_INFO_ONCE("Starting detection processs");

        // update thresImage
        thresholding();
        // update target pixels
        if (not uploadTargetPixel() ){
            ROS_WARN("lost target.");
            return false;
        }
        cl->pntPixelQuery(curTargetPixels); // upload the pixel to cl
        imshow(window_name, thresImage); //show the thresholded image
        cv::waitKey(1);
        return true;
    }

}
/**
 * Update the target pixels by binary thres holding
 */
bool TargetHSV::uploadTargetPixel() {
    if (cv::countNonZero(thresImage) == 0 ){
        ROS_WARN("Target is not detected! ");
        return false;
    }else{
        curTargetPixels.clear();
        for (int r = 0 ; r < thresImage.rows ; r++ ){
            auto rowPtr = thresImage.ptr<uchar>(r);
            for (int c = 0 ; c < thresImage.cols ; c++ ){
                if (rowPtr[c] == 255)
                    curTargetPixels.push_back(cv::Point(c,r));
            }
        }

//        cout << "[HSV detector] current number of target pixels  "<< curTargetPixels.size() << endl;


        return true;
    }


}

void TargetHSV::run(){
    ros::Rate rl(30);
    while(ros::ok()){
        if (update()){

        } else{
            ROS_WARN_THROTTLE(2,"[TargetHSV] update failed.");
        }
        publish();
        cl->publish();
        ros::spinOnce();
        rl.sleep();
    }
}


