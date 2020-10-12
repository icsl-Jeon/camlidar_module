//
// Created by jbs on 20. 10. 12..
//

#include "TargetHSV.h"

int main(int argc,char** argv){

    ros::init(argc,argv,"target_hsv_extractor");
    TargetHSV targetHSV;
    targetHSV.run();

}