#ifndef __JHR_MOTO__
#define __JHR_MOTO__
#include <iostream>
#include     <stdio.h>
#include     <stdlib.h>
#include     <unistd.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#ifndef uint8
#define uint8 unsigned char
#endif
#ifndef uint16
#define uint16 unsigned short
#endif
#pragma pack(1)
//底盘马达驱动器返回数据结构
typedef struct{
    //0x9bc2,低字节在前
    uint16 head;
    //状态:1：霍尔故障,2:电机堵转,3:电机过流,4:硬件过流,5:通讯异常,6:速度异常
    //高四位为电机1状态，低四位为电机2状态
    uint8 state;
    //电机1脉冲个数值
    int lData;
    //电机2脉冲个数值
    int rData;
}MotoRcv;


#pragma pack()
class Jhr_moto
{
public:
    //静态成员函数目的是访问静态成员变量。
    //静态成员变量只有一个，无论多少个实例化对象，其所有对象共享一个静态成员变量
    static void moto_init(void);//电机初始化
    static void clear_error();  //清除驱动错误
    static void moto_lock();	//锁轮工作模式，马达停止时锁轮
    static void moto_free();  //自由工作模式，马达停止时不锁轮
    //设置左右轮速度运行
    static void setSpeed(short lSpeed,short rSpeed);
    //两轮差速运动学部分，将线速度，角速度转化为两轮的差速
    static void setSpeedRos(float linear,float angular);
    //原地旋转指定角度
    static void setTurnMode(float angle);
    //直线运行指定距离
    static void setLineMode(float);
    //获取里程计 X
    static float getX();
    //获取里程计 Y
    static float getY();
    //获得马达错误状态
    static uint8 getState();
    //获取里程计 角度
    static float getRadian();
    static float getLinear();
    static float getAngular();
    //设置左右轮间距
    static void setWheelSpacing(short w_s);
    static void setPulseEquivalent(float pulse_equivalent);
    static void jhr_file_record(const std::string file_name,const std::string msg,int index=0);
    // 主回调函数，负责里程计的计算与odom发布
    static void rcv_callback_1(const char*buf,int len);
    static void moto_loop();  //循环调用通讯函数
    static bool is_ask; //是否收到回复消息
    static void tw_callback(const geometry_msgs::Twist &tw);
    static void odom_publish(void);

    static int kp;
    static int ki;
    static int type;    //马达类型:0,第一种马达，1：和利时双驱动马达
    static short lSpeed;
    static short rSpeed;
    static float vLinear;
    static float vAngular;
    static float vLinear0;
    static float vAngular0;
    static unsigned char state;
    static ros::Publisher *odom_pub_ptr;
    static tf::TransformBroadcaster *odom_broadcaster;
    static short wheel_spacing;//轮间距
    static float pulse_equivalent;//脉冲弧度
    static float posX,posY,posRadian;
    static int work_mode;

    static void TurnMode_handle();
    static void LineMode_handle();
        
    static unsigned char ac;// 加速度
    static unsigned char cmd;
    static unsigned char cmd_1;
    
    static int lDistance;//距离
    static int rDistance;
    
};

#endif