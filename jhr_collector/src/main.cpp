#include "ros/ros.h"
#include <sstream>
#include <std_msgs/Int32.h>
#include "jhr_udp.h"
#include "jhr_moto.h"
#include "jhr_moto_hls.h"

ros::NodeHandle *n_ptr = NULL;

int main(int argc,char**argv)
{
    std::string dev_ip;
    std::string cmd_node;
    int wheel_spacing;
    int pulse_equivalent;
    ros::init(argc, argv, "jhr_collector");
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("cmd_node", cmd_node, "/cmd_vel");
    nh_private.param<std::string>("dev_ip", dev_ip, "192.168.11.104");
    nh_private.param<int>("wheel_spacing", wheel_spacing, 308);
    nh_private.param<int>("pulse_equivalent", pulse_equivalent, 7670);
    nh_private.param<int>("moto_kp", Jhr_moto::kp, 0);
    nh_private.param<int>("moto_ki", Jhr_moto::ki, 15);
    nh_private.param<int>("moto_type", Jhr_moto::type, 0);  //0：

    Jhr_moto::setSpeedRos(0,0);	//设置初始速度为0
    Jhr_moto::setWheelSpacing(wheel_spacing);	//设置轮间距390mm
    Jhr_moto::setPulseEquivalent(pulse_equivalent/1000.0f); //设置每毫米脉冲数:7.67 = 4096 / 170 / 3.1416

    ros::NodeHandle n;
    n_ptr = &n;
    //$ 当收到cmd_node后，将进入jhr_moto::tw_callback()函数进行处理，通过差速运动学得到rSpeed_r，lSpeed_r
    ros::Subscriber tw_sub = n.subscribe(cmd_node,1,Jhr_moto::tw_callback);
    ros::Publisher odom_pub =  n.advertise<nav_msgs::Odometry>("wheel_odom", 1000);
    Jhr_moto::odom_pub_ptr = &odom_pub;
    
    Jhr_moto::odom_broadcaster = new tf::TransformBroadcaster();
    ros::Rate loop_rate(10); 
    Jhr_udp::udp_poll_init(dev_ip.c_str());
    Jhr_udp::jhr_udp_arr[0]->set_rcv_callback(NULL);
    Jhr_udp::jhr_udp_arr[1]->set_rcv_callback(NULL);
    Jhr_udp::jhr_udp_arr[2]->set_rcv_callback(Jhr_moto_hls::rcv_callback_1);
    Jhr_udp::jhr_udp_arr[3]->set_rcv_callback(NULL);
    Jhr_udp::jhr_udp_arr[4]->set_rcv_callback(NULL);
    Jhr_udp::jhr_udp_arr[5]->set_rcv_callback(NULL);

    while (ros::ok())
    {
        Jhr_moto_hls::moto_loop();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}