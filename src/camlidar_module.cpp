//
// Created by jbs on 20. 10. 12..
//

# include <camlidar_sync_align.h>
#include <tf/transform_listener.h>


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

/*
*
*
*
*
*/
/* implementation */
CamLidarSyncAlign::CamLidarSyncAlign(ros::NodeHandle& nh, const string& param_path_, const string& save_path)
        : nh_(nh), save_dir_(save_path)
{
    cout << " ALGINER STARTS.\n";

    // initialize image container & subscribers.
    topicname_img_ = "/0/image_raw";

    // initialize lidar container & subscribers.
    buf_lidar_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    buf_lidar_x = new float[100000];
    buf_lidar_y = new float[100000];
    buf_lidar_z = new float[100000];

    buf_lidar_x_warped = new float[100000];
    buf_lidar_y_warped = new float[100000];
    buf_lidar_z_warped = new float[100000];

    buf_lidar_u_projected = new float[100000];
    buf_lidar_v_projected = new float[100000];

    buf_lidar_intensity = new float[100000];
    buf_lidar_ring = new unsigned short[100000];
    buf_lidar_time = new float[100000];
    n_pts_lidar = 0;

    topicname_lidar_ = "/lidar0/velodyne_points";

    this->pubBBQueriedPCL = nh.advertise<sensor_msgs::PointCloud2>("bb_queried_pnts",1);
    this->pubBBQueriedPCLProcess = nh.advertise<sensor_msgs::PointCloud2>("bb_queried_pnts_processed",1);

    this->pubVelodyneOriginalTime = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_original",1);
    this->pubCaliImage = nh.advertise<sensor_msgs::Image>("cali_image",1);

    nh.param<string>("map_frame",map_frame,"map");
    nh.param("dbscan_eps",dbscan_eps,0.1f);
    nh.param("dbscan_min_pnts",dbscan_min_pnts,3);


    ros::param::get("~baselink_id", baselink_frame);

//    this->baselink_frame = nh.param<string>("baselink_id",baselink_frame,"base_link");
    this->img_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, topicname_img_, 1);
    this->lidar_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, topicname_lidar_, 1);

    // Generate topic synchronizer
    this->sync_sub = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*this->img_sub, *this->lidar_sub);
    this->sync_sub->registerCallback(boost::bind(&CamLidarSyncAlign::callbackImageLidarSync,this, _1, _2));

    // Load calibration parameters
    readCameraLidarParameter(param_path_);
    preCalculateUndistortMaps();

    // snapshot parameters
    current_seq_ = 0;
    data_ready_ = false;




    // generate save folder
//    std::string folder_create_command;
//    folder_create_command = "sudo rm -rf " + save_dir_;
//	system(folder_create_command.c_str());
//    folder_create_command = "mkdir " + save_dir_;
//	system(folder_create_command.c_str());
//
//    // make image saving directories
//    folder_create_command = "mkdir " + save_dir_ + "cam0/";
//	system(folder_create_command.c_str());
//
//    // make lidar data saving directories
//    folder_create_command = "mkdir " + save_dir_ + "lidar0/";
//	system(folder_create_command.c_str());
//
//    // save association
//    string file_name = save_dir_ + "/association.txt";
//    std::ofstream output_file(file_name, std::ios::trunc);
//    output_file.precision(6);
//    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
//    if(output_file.is_open()){
//        output_file << "time_us ";
//        output_file << "cam0 ";
//        output_file << "exposure_us gain_dB ";
//        output_file << "lidar0 ";
//        output_file << "\n";
//    }

    // debug image on/off
    flag_debugimage_ = true;
    ros::param::get("~flag_debugimage", flag_debugimage_);

    if(flag_debugimage_){
        winname_ = "undistorted image";
         cv::namedWindow(winname_,cv::WINDOW_AUTOSIZE);
    }

    // index image
    img_index_ = cv::Mat::zeros(n_rows, n_cols, CV_16S);

    // BB queried points
    lastQueriedPoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    lastQueriedPointsProcess = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);



    T_bl.setZero();
    T_bl(3,3) = 1.0;
    T_bl(1,0) =  1.0;
    T_bl(0,1) =  1.0;
    T_bl(2,2) =  -1.0;


};

CamLidarSyncAlign::~CamLidarSyncAlign(){
    delete[] buf_lidar_x;
    delete[] buf_lidar_y;
    delete[] buf_lidar_z;

    delete[] buf_lidar_x_warped;
    delete[] buf_lidar_y_warped;
    delete[] buf_lidar_z_warped;

    delete[] buf_lidar_u_projected;
    delete[] buf_lidar_v_projected;

    delete[] buf_lidar_intensity;
    delete[] buf_lidar_ring;
    delete[] buf_lidar_time;
};


void CamLidarSyncAlign::publish() {
    if(isReceived and pubVelodyneOriginalTime.getNumSubscribers() > 0)
        pubVelodyneOriginalTime.publish(receivedVelodyne);

    if(isReceived and pubCaliImage.getNumSubscribers() > 0)
        pubCaliImage.publish(imageToROSmsg(img_cali_,sensor_msgs::image_encodings::BGR8,"bluefox",ros::Time::now()));

    if (isBBQueried and isQueryValid){

        ROS_INFO_ONCE("queried points publishing starts.");
        sensor_msgs::PointCloud2 pclROS;
        sensor_msgs::PointCloud2 pclROSProcess;

        pcl::toROSMsg(*lastQueriedPoints,pclROS);
        pcl::toROSMsg(*lastQueriedPointsProcess,pclROSProcess);

        pclROS.header.stamp = ros::Time::now();
        pclROS.header.frame_id = baselink_frame;
        pclROSProcess.header.stamp = ros::Time::now();
        pclROSProcess.header.frame_id = baselink_frame;

        pubBBQueriedPCL.publish(pclROS);
        pubBBQueriedPCLProcess.publish(pclROSProcess);
    }




}

void CamLidarSyncAlign::callbackImageLidarSync(const sensor_msgs::ImageConstPtr& msg_image, const sensor_msgs::PointCloud2ConstPtr& msg_lidar){
    isReceived = true;
    // get image
    //cout << " camlidar node callback.\n";
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);
    buf_img_ = cv_ptr->image;

    // undistort image
    this->undistortCurrentImage(buf_img_, img_undistort_);
//    baselink_frame = msg_lidar->header.frame_id;



    double time_img = (double)(msg_image->header.stamp.sec * 1e6 + msg_image->header.stamp.nsec / 1000) / 1000000.0;
    double time_lidar = (double)(msg_lidar->header.stamp.sec * 1e6 + msg_lidar->header.stamp.nsec / 1000) / 1000000.0;

    receivedVelodyne = *msg_lidar;
    receivedVelodyne.header.stamp = ros::Time::now(); // time reconfigure to orignal ros time

    buf_time_ = time_img;

    // get width and height of 2D point cloud data
    for(int i = 0; i < msg_lidar->width; i++) {
        int arrayPosX = i*msg_lidar->point_step + msg_lidar->fields[0].offset; // X has an offset of 0
        int arrayPosY = i*msg_lidar->point_step + msg_lidar->fields[1].offset; // Y has an offset of 4
        int arrayPosZ = i*msg_lidar->point_step + msg_lidar->fields[2].offset; // Z has an offset of 8

        int ind_intensity = i*msg_lidar->point_step + msg_lidar->fields[3].offset; // 12
        int ind_ring = i*msg_lidar->point_step + msg_lidar->fields[4].offset; // 16
        int ind_time = i*msg_lidar->point_step + msg_lidar->fields[5].offset; // 18

        float X = 0.0;
        float Y = 0.0;
        float Z = 0.0;
        float intensity = 0.0;
        unsigned short ring = 0.0;
        float time = 0.0;

        memcpy(buf_lidar_x+i, &msg_lidar->data[arrayPosX], sizeof(float));
        memcpy(buf_lidar_y+i, &msg_lidar->data[arrayPosY], sizeof(float));
        memcpy(buf_lidar_z+i, &msg_lidar->data[arrayPosZ], sizeof(float));
        memcpy(buf_lidar_intensity+i, &msg_lidar->data[ind_intensity], sizeof(float));
        memcpy(buf_lidar_ring+i, &msg_lidar->data[ind_ring], sizeof(unsigned short));
        memcpy(buf_lidar_time+i, &msg_lidar->data[ind_time], sizeof(float));
    }
    n_pts_lidar = msg_lidar->width; // # of lidar points in one circle

    // warp and project lidar points
    warpAndProjectLidarPointcloud();


    if(flag_debugimage_){
        cv::Scalar orange(0, 165, 255), blue(255, 0, 0), magenta(255, 0, 255);

        cv::Mat img_8u;
        img_undistort_.convertTo(img_8u, CV_8UC3);
//        cout << "n_pts_warped: "<<n_pts_lidar_warped<<"\n";
        for(int i = 0; i < n_pts_lidar_warped; i++){
            cv::Point pt(buf_lidar_u_projected[i], buf_lidar_v_projected[i]);
            cv::circle(img_8u, pt, 1, magenta);
        }

        cv::rectangle(img_8u,lastBBQuery,cv::Scalar(0,255,0));
        img_cali_ = img_8u;
        cv::imshow(winname_, img_8u);
        cv::waitKey(10);
    }

    // Create a container for the output lidar data.
    sensor_msgs::PointCloud2 output;

    // Do data processing here...
    output = *msg_lidar;


    pcl::PointCloud<pcl::PointXYZI>::Ptr temp = buf_lidar_;
    temp->clear();
    pcl::fromROSMsg(output, *temp);

    int n_pts = temp->points.size();


    //cout << "Callback time (image): " << time_img << "\n";
    //cout << "Callback time (lidar): " << time_lidar << "\n";
    //cout <<"n_pts lidar: " <<n_pts<<endl;

    data_ready_ = true;
};
/**
 * Points query on undistorted image !
 * @param boundingBox
 * @return
 */
void CamLidarSyncAlign::pntBBQuery(cv::Rect boundingBox) {
    lastBBQuery = boundingBox;
    lastQueriedPoints->clear();
    isBBQueried = true;

//    cout << img_index_ << endl;
    for (int r = boundingBox.y ; r < boundingBox.y + boundingBox.height ; r++ ) {

        const short* ptr = img_index_.ptr<short >(r);

        for (int c = boundingBox.x; c < boundingBox.x + boundingBox.width; c++) {
            short idx = ptr[c];
//            cout << "index: " << idx << endl;
            if (idx > 0) {
                double x = buf_lidar_x_warped[idx];
                double y = buf_lidar_y_warped[idx];
                double z = buf_lidar_z_warped[idx];

                Eigen::Vector4d xyz_velo = T_bl*T_cl.inverse()*Eigen::Vector4d(x,y,z,1); // w.r.t velodyne

                pcl::PointXYZ pnt;
                pnt.x = xyz_velo(0);
                pnt.y = xyz_velo(1);
                pnt.z = xyz_velo(2);
                lastQueriedPoints->push_back(pnt);

            }
        }
    }
}

/**
 * Points query on undistorted image !
 * @param boundingBox
 * @return
 */
void CamLidarSyncAlign::pntPixelQuery(const vector<cv::Point>& queryPntSet) {
    lastQueriedPoints->clear();
    lastQueriedPointsProcess->points.clear();

    isQueryValid = (queryPntSet.size() != 0);

    if (isQueryValid) {
        vector<dbscan::Point> dbscanPoints;
        //    cout << img_index_ << endl;
        for (auto pnt : queryPntSet) {
            int r = pnt.y;
            int c = pnt.x;
            const short *ptr = img_index_.ptr<short>(r);
            short idx = ptr[c];
            if (idx > 0) {
                double x = buf_lidar_x_warped[idx];
                double y = buf_lidar_y_warped[idx];
                double z = buf_lidar_z_warped[idx];
                Eigen::Vector4d xyz_velo = T_bl * T_cl.inverse() * Eigen::Vector4d(x, y, z, 1); // w.r.t velodyne
                pcl::PointXYZ pnt;
                pnt.x = xyz_velo(0);
                pnt.y = xyz_velo(1);
                pnt.z = xyz_velo(2);
                lastQueriedPoints->push_back(pnt);
                // dbscan points
                dbscan::Point DBpnt;
                DBpnt.x = pnt.x;
                DBpnt.y = pnt.y;
                DBpnt.z = pnt.z;
                DBpnt.clusterID = UNCLASSIFIED;
                dbscanPoints.push_back(DBpnt);
            }
        }


        // Clustering by dbscan
        dbscan::DBSCAN dbscanObj(3, 0.1, dbscanPoints);
        dbscanObj.run();
        int totalClusterId = 0; // start from 1
        vector<dbscan::PointSet> pointSet(100); // initialize with enough clustering numbers

        for (int i = 0; i < dbscanPoints.size(); i++) {
            int curCluster = dbscanObj.m_points[i].clusterID;
            if (curCluster >=1)
            pointSet[curCluster-1].points.push_back(dbscanObj.m_points[i]);
            totalClusterId = max(totalClusterId, curCluster);
        }
        Eigen::MatrixXf centerMat(totalClusterId,3);
        for (int i = 0 ; i < totalClusterId ; i++){
            dbscan::Point curCenter = pointSet[i].getCenter(); // w.r.t frame id of  incoming pcl (baselink_frame)
            tf::Stamped<tf::Point> pPcl; pPcl.frame_id_ = baselink_frame;
            tf::Stamped<tf::Point> pMap; pMap.frame_id_ = map_frame;
            pPcl.setData(tf::Point(curCenter.x,curCenter.y,0));
            tf_l.transformPoint(map_frame,ros::Time(0),pPcl,baselink_frame,pMap);
            centerMat.block(i,0,1,3) = Eigen::Vector3f(curCenter.x,curCenter.y,curCenter.z).transpose();
        }

        if (totalClusterId > 1) {
            ROS_WARN_THROTTLE(2, "target pcl has multiple clusters! (%d) \n We pick the closest\n ", totalClusterId);
            ROS_WARN_STREAM_THROTTLE(2,centerMat);
        }


        Eigen::Vector3f refPoint;

        if(not isBBQueried){
            // should do some initialization on the position of the target
            // we assume the target is the nearest pcl to the origin

            refPoint = Eigen::Vector3f::Zero();
        }else{
            // if previously solved, pick the nearest cluster
            refPoint = prevTargetClusterCenter;
        }

        double minDistToOrigin = 1E+6;
        int targetClusterId = -1;
        for (int n = 0 ; n < totalClusterId ; n++){
            double dist = (centerMat.block(n,0,1,3).transpose() - refPoint).norm();
            if (dist < minDistToOrigin){
                minDistToOrigin = dist;
                targetClusterId = n;
                prevTargetClusterCenter = centerMat.block(n,0,1,3).transpose();
            }
        }
        ROS_INFO_ONCE("Initialized the center of target cloud as [%f,%f] from %d clusters.\n ",prevTargetClusterCenter(0),prevTargetClusterCenter(1),totalClusterId);

        // save it to processed target pcl

        if (targetClusterId >= 0) {


            for (auto pnt : pointSet[targetClusterId].points) {
                pcl::PointXYZ pntPCL;
                pntPCL.x = pnt.x;
                pntPCL.y = pnt.y;
                pntPCL.z = pnt.z;
                lastQueriedPointsProcess->points.push_back(pntPCL);
            }
        }else{
            ROS_WARN("seems no clusters existing..");
        }
        isBBQueried = true;
    }
}





void CamLidarSyncAlign::readCameraLidarParameter(const string& path_dir){
    // Load calibration files
    cv::FileStorage fsSettings(path_dir, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
        cerr << "ERROR: Wrong path to settings" << endl;

    fsSettings["IMAGE.K"] >> cvK;
    fsSettings["IMAGE.DISTORTION"] >> cvDistortion;
    fsSettings["T_CL"] >> cvT_cl;
    this->n_rows = fsSettings["IMAGE.ROWS"];
    this->n_cols = fsSettings["IMAGE.COLS"];

    cout << cvK<<endl;
    cout << cvDistortion<<endl;
    cout<<cvT_cl<<endl;
    cout<<"n_rows: "<<this->n_rows<<", n_cols: "<<this->n_cols<<endl;

    fsSettings.release();

    if (cvK.empty() || cvDistortion.empty() || cvT_cl.empty() || n_rows == 0 || n_cols == 0)
        cerr << "ERROR: Calibration parameters to rectify an image are missing!" << endl;

    // Eigen version
    cv::cv2eigen(cvT_cl, T_cl);
    cv::cv2eigen(cvK, K);

    cout << "  Camera & LiDAR information is loaded... DONE!\n";
};

void CamLidarSyncAlign::preCalculateUndistortMaps(){
    // allocation
    undist_map_x   = cv::Mat::zeros(n_rows, n_cols, CV_32FC1);
    undist_map_y   = cv::Mat::zeros(n_rows, n_cols, CV_32FC1);
    img_undistort_ = cv::Mat::zeros(n_rows, n_cols, CV_8UC3);


    // interpolation grid calculations
    float* map_x_ptr = nullptr;
    float* map_y_ptr = nullptr;

    float fu = K(0,0);
    float fv = K(1,1);
    float fuinv = 1.0f/fu;
    float fvinv = 1.0f/fv;
    float centeru = K(0,2);
    float centerv = K(1,2);
    float k1 = cvDistortion.at<double>(0,0);
    float k2 = cvDistortion.at<double>(0,1);
    float p1 = cvDistortion.at<double>(0,2);
    float p2 = cvDistortion.at<double>(0,3);
    float k3 = cvDistortion.at<double>(0,4);

    for(int v = 0; v < n_rows; v++){
        map_x_ptr = undist_map_x.ptr<float>(v);
        map_y_ptr = undist_map_y.ptr<float>(v);

        for(int u = 0; u < n_cols; u++){
            float x = (u-centeru)*fuinv;
            float y = (v-centerv)*fvinv;
            float r2 = x*x + y*y;
            float r4 = r2*r2;
            float r6 = r2*r4;
            float r = sqrtf(r2);
            float r_radial = 1.0f + k1*r2 + k2*r4 + k3*r6;
            float x_dist = x*r_radial + 2*p1*x*y + p2*(r2 + 2*x*x);
            float y_dist = y*r_radial + p1*(r2 + 2*y*y) + 2*p2*x*y;
            *(map_x_ptr+u) = fu*x_dist + centeru;
            *(map_y_ptr+u) = fv*y_dist + centerv;
        }
    }
};

void CamLidarSyncAlign::undistortCurrentImage(const cv::Mat& img_source, cv::Mat& img_dst){
    cv::remap(img_source, img_dst, this->undist_map_x, this->undist_map_y, CV_INTER_LINEAR);
};

void CamLidarSyncAlign::saveLidarDataRingTime(const string& file_name){
    int n_pts = n_pts_lidar;

    std::ofstream output_file(file_name, std::ios::trunc);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);

    if(output_file.is_open()){
        output_file << "# .PCD v.7 - Point Cloud Data file format\n";
        output_file << "VERSION .7\n";
        output_file << "FIELDS x y z intensity ring time\n";
        output_file << "SIZE 4 4 4 4 2 4\n";
        output_file << "TYPE F F F F U F\n";
        output_file << "COUNT 1 1 1 1 1 1\n";
        output_file << "WIDTH " << n_pts << "\n";
        output_file << "HEIGHT 1\n";
        output_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        output_file << "POINTS " << n_pts<< "\n";
        output_file << "DATA ascii\n";
        for(int i = 0; i < n_pts; i++){
            output_file << *(buf_lidar_x + i)<<" ";
            output_file << *(buf_lidar_y + i)<<" ";
            output_file << *(buf_lidar_z + i)<<" ";
            output_file << *(buf_lidar_intensity + i)<<" ";
            output_file << *(buf_lidar_ring + i)<<" ";
            output_file << *(buf_lidar_time + i)<<"\n";
        }
    }
};

void CamLidarSyncAlign::saveSnapshot(){
    // save images
    bool static png_param_on = false;
    vector<int> static png_parameters;
    if (png_param_on == false)
    {
        png_parameters.push_back(cv::IMWRITE_PNG_COMPRESSION); // We save with no compression for faster processing
        png_parameters.push_back(0);
        png_param_on = true;
    }
    ++current_seq_;
    string file_name = save_dir_ + "/cam0/" + itos(current_seq_) + ".png";
    cv::imwrite(file_name, buf_img_, png_parameters);

    // save lidars
    file_name = save_dir_ + "/lidar0/" + itos(current_seq_) + ".pcd";
    saveLidarDataRingTime(file_name);

    // save association
    file_name = save_dir_ + "/association.txt";
    std::ofstream output_file(file_name, std::ios::app);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
    if(output_file.is_open()){
        output_file << buf_time_ << " ";
        output_file << "/cam0/" << current_seq_ << ".png ";
        output_file << "/lidar0/" << current_seq_ << ".pcd ";
        output_file << "\n";
    }
};

void CamLidarSyncAlign::warpAndProjectLidarPointcloud(){
    n_pts_lidar_warped = 0;

    float y_min = 0.0;
    float y_max = 20.0;

    // fill img_index_ with -1;
    img_index_ = -1;

    // warp point via T_cl
    Eigen::Vector3d X, X_warped, X_projected;
    for (int i = 0; i < n_pts_lidar; i++){
        X(0) = buf_lidar_x[i];
        X(1) = buf_lidar_y[i];
        X(2) = buf_lidar_z[i];
        X_warped = T_cl.block<3,3>(0,0)*X + T_cl.block<3,1>(0,3);
        buf_lidar_x_warped[i] = X_warped(0);
        buf_lidar_y_warped[i] = X_warped(1);
        buf_lidar_z_warped[i] = X_warped(2);

        // projection
        X_projected(0) = X_warped(0)/X_warped(2)*K(0,0)+K(0,2);
        X_projected(1) = X_warped(1)/X_warped(2)*K(1,1)+K(1,2);
        if(X(1) > y_min & X(1) < y_max & X_projected(0) > 0 & X_projected(0) < n_cols & X_projected(1) > 0 & X_projected(1) < n_rows){
            buf_lidar_u_projected[n_pts_lidar_warped] = X_projected(0);
            buf_lidar_v_projected[n_pts_lidar_warped] = X_projected(1);
            ++n_pts_lidar_warped;

            // fill "img_index_"
            short* ptr_temp = img_index_.ptr<short>(0);
            short u,v;
            u = (short)X_projected(0);
            v = (short)X_projected(1);
            *(ptr_temp + n_cols*v + u) = i;

        }
    }
};
