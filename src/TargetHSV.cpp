//
// Created by jbs on 20. 10. 12..
//

#include "TargetHSV.h"



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
    nh.param("imshow",setImshow,true);
    nh.param("toCenterRatio",toCenterRatio,0.5f);


    pubThresMask= it.advertise("undistorted_thres_mask",1);
    pubThresMaskProcess = it.advertise("undistorted_thres_mask_process",1);

    if (setImshow)
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

    cvtColor(thresImage,thresImageProcess,cv::COLOR_GRAY2BGR);
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
    pubThresMask.publish(imageToROSmsg(thresImage,sensor_msgs::image_encodings::MONO8,"bluefox",ros::Time::now()));
    pubThresMaskProcess.publish(imageToROSmsg(thresImageProcess,sensor_msgs::image_encodings::BGR8,"bluefox",ros::Time::now()));
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
            vector<cv::Point> emptySet;
            cl->pntPixelQuery(emptySet);
            ROS_WARN("lost target.");
            return false;
        }
        cl->pntPixelQuery(curTargetPixels); // upload the pixel to cl
        if (setImshow){
            imshow(window_name, thresImage); //show the thresholded image
            cv::waitKey(1);
        }

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

        vector<float> rs;
        vector<float> cs;

        for (int r = 0 ; r < thresImage.rows ; r++ ){
            auto rowPtr = thresImage.ptr<uchar>(r);
            for (int c = 0 ; c < thresImage.cols ; c++ ){
                if (rowPtr[c] == 255) {
                    curTargetPixels.push_back(cv::Point(c, r));
                    rs.push_back(r);
                    cs.push_back(c);
                }
            }
        }

        // get the center
        float rCenter = accumulate(rs.begin(),rs.end(),0.0)/rs.size();
        float cCenter = accumulate(cs.begin(),cs.end(),0.0)/cs.size();

        curTargetPixelCenter = cv::Point(int(cCenter),int(rCenter));
        cv::Rect2i centerBox(curTargetPixelCenter,cv::Size(2,2));

        //        cout << "[HSV detector] current number of target pixels  "<< curTargetPixels.size() << endl;

        // shrink toward the center
        for (auto &pnt : curTargetPixels){

            pnt.x = int((1-toCenterRatio)*pnt.x + toCenterRatio*curTargetPixelCenter.x);
            pnt.y = int((1-toCenterRatio)*pnt.y + toCenterRatio*curTargetPixelCenter.y);
            cv::rectangle(thresImageProcess,cv::Rect(pnt.x,pnt.y,1,1),
                          cv::Scalar(0,255,0),2);

        }
        cv::rectangle(thresImageProcess,centerBox,cv::Scalar(0,0,255),2);

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


