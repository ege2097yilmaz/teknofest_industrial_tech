#include <ros/ros.h>
#include <iostream>
#include <teknofest_industrial_tech/qr.h>
#include <ros/console.h>

int main(int argc, char **argv){

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) { // Change the level to fit your needs
    ros::console::notifyLoggerLevelsChanged();
    }

    int qr_number;
    bool qr_state;
    ros::init(argc, argv, "qr_publisher_node");
    ROS_INFO("qr_publisher is starting ...");
    ros::NodeHandle n;

    ros::Publisher qr_pub = n.advertise<teknofest_industrial_tech::qr>("/qr_state", 1000);
    ros::Rate loop_rate(10);

    while(ros::ok()){

        std::cout<<"qr_state input"<<std::endl;
        std::cin>>qr_number;
        
        if(qr_number == 1){
            qr_state = true;
        }else qr_state = false;

        if(qr_state){

            ROS_DEBUG("button is detected");
            teknofest_industrial_tech::qr qr;
            qr.qr = true;
            qr.qr_name = "qr state";
            qr.stamp = ros::Time::now();
            qr_pub.publish(qr);
            qr_state = false;}
            
        else{
            ROS_WARN("button is not detected");
            }
    }
}