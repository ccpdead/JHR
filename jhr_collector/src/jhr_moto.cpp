#include "jhr_moto.h"
#include "jhr_udp.h"
#include <math.h>
#include <sstream>

short Jhr_moto::lSpeed = 0; //左右轮经过差速运动学后计算得到的差速
short Jhr_moto::rSpeed = 0; //左右轮经过差速运动学后计算得到的差速

int Jhr_moto::kp = 0;  //(5~250)default:20
int Jhr_moto::ki = 15; //(1:50)default:2
int Jhr_moto::type = 0;
unsigned char Jhr_moto::ac = 0x18; //(1:250)default:0x18
unsigned char Jhr_moto::cmd = 2;
unsigned char Jhr_moto::cmd_1 = 2;
unsigned char Jhr_moto::state = 0;
int Jhr_moto::lDistance = 0;
int Jhr_moto::rDistance = 0;
short Jhr_moto::wheel_spacing = 390;
float Jhr_moto::pulse_equivalent = 8.1f; //每个脉冲所表示的弧度值
float Jhr_moto::posX = 0;
float Jhr_moto::posY = 0;
float Jhr_moto::posRadian = 0;
float Jhr_moto::vLinear = 0;                            //？
float Jhr_moto::vAngular = 0;                           //？
float Jhr_moto::vLinear0 = 0;                           //记录jhr_motor::tw_callback()中收到的线速度
float Jhr_moto::vAngular0 = 0;                          //记录jhr_motor::tw_callback()中收到的角速度
bool Jhr_moto::is_ask = true;                           //运行标志位
int Jhr_moto::work_mode = 0;                            //工作模式  0:速度模式，1：直线运行,2:原地旋转
float posX_start, posY_start;                           //直线运行、原地旋转模式的起始值
float turn_Radian_set, posRadian_old, turn_Radian_curr; //原地旋转模式 旋转弧度设定值,旧值，当前值，
float line_distance;                                    //直线运行
ros::Time curr_time;
ros::Publisher *Jhr_moto::odom_pub_ptr = NULL;
tf::TransformBroadcaster *Jhr_moto::odom_broadcaster = NULL;

void Jhr_moto::moto_init(void)
{
}

/**
 * 向指定文件写入信息
 */
void Jhr_moto::jhr_file_record(const std::string file_name, const std::string msg, int index)
{
    FILE *fp;
    if (index == 0)
    {
        fp = fopen(file_name.c_str(), "a");
    }
    else
    {
        fp = fopen(file_name.c_str(), "w");
    }
    if (fp == NULL)
    {
        // fprintf(stderr,"jhr_file_record fopen %d:%s %s\n",errno,strerror(errno),file_name.c_str());
        return;
    }
    fputs(msg.c_str(), fp);
    fclose(fp);
}

void Jhr_moto::setWheelSpacing(short w_s)
{
    wheel_spacing = w_s;
}

void Jhr_moto::setPulseEquivalent(float pulseEquivalent)
{
    pulse_equivalent = pulseEquivalent;
}

void Jhr_moto::setSpeed(short _lSpeed, short _rSpeed)
{
    lSpeed = _lSpeed;
    rSpeed = _rSpeed;
}
//原地旋转指定角度
void Jhr_moto::setTurnMode(float radian)
{
    work_mode = 2;
    turn_Radian_set = radian,
    posRadian_old = posRadian;
    turn_Radian_curr = 0;

    float angular;
    if (radian > 0)
        angular = 0.15;
    else
        angular = -0.15;
    lSpeed = angular * wheel_spacing / 2 * -1;
    rSpeed = lSpeed;
    is_ask = true;
}

/**
 * @brief 该函数在jhr_moto_hls::motor_loop()中被调用。作用？
 *
 */
void Jhr_moto::TurnMode_handle()
{
    if (work_mode != 2)
        return;
    float a1 = posRadian - posRadian_old;
    posRadian_old = posRadian;
    if (a1 > (3.1415926))
        a1 -= (3.1415926 * 2);
    if (a1 < (-3.1415926))
        a1 += (3.1415926 * 2);
    turn_Radian_curr += a1;
    float f1 = (turn_Radian_curr > 0 ? turn_Radian_curr : turn_Radian_curr * -1);
    float f2 = (turn_Radian_set > 0 ? turn_Radian_set : turn_Radian_set * -1);
    if (f1 >= f2)
    {
        work_mode = 0;
        lSpeed = 0.0f;
        rSpeed = 0.0f;
    }
    is_ask = true;
    std::cout << "Turn_mode:" << turn_Radian_set << "  " << turn_Radian_curr << "  " << a1 << std::endl;
}

/**
 * @brief 按照直线行驶一定距离
 *
 * @param distance
 */
void Jhr_moto::setLineMode(float distance)
{
    work_mode = 1;
    posX_start = posX;
    posY_start = posY;
    line_distance = distance * 1000;
    if (distance > 0)
    {
        lSpeed = 0.1f * 1000;
    }
    else
    {
        lSpeed = -0.1f * 1000;
    }
    rSpeed = lSpeed * -1;
    is_ask = true;
}

/**
 * @brief 该函数在jhr_moto_hls::motor_loop()中被调用，作用？
 *
 */
void Jhr_moto::LineMode_handle(void)
{
    //直线运行模式
    if (work_mode != 1)
        return;
    float f1 = sqrt((posX_start - posX) * (posX_start - posX) + (posY_start - posY) * (posY_start - posY));
    float f2 = (line_distance > 0 ? line_distance : line_distance * -1);
    if (f1 >= f2)
    {
        work_mode = 0;
        lSpeed = 0.0f;
        rSpeed = 0.0f;
    }
    is_ask = true;
    std::cout << "line_mode:" << f2 << " " << f1 << " " << posX << " " << posY << std::endl;
}

/**
 * @brief 差速运动学，将线速度，角速度转化为左右电机差速
 *
 * @param linear
 * @param angular
 */
void Jhr_moto::setSpeedRos(float linear, float angular)
{
    if (work_mode != 0)
        return; // 0 为速度模式
    linear *= 1000;
    vLinear0 = linear;
    vAngular0 = angular;
    if (angular == 0)
    {
        lSpeed = linear;
        rSpeed = linear * -1;
    }
    else if (linear == 0)
    {
        lSpeed = angular * wheel_spacing / 2 * -1;
        rSpeed = lSpeed;
    }
    else
    {
        float fR = linear / angular;
        float f_1 = (fR + wheel_spacing / 2) / (fR - wheel_spacing / 2);
        lSpeed = linear * 2 / (1 + f_1);
        rSpeed = f_1 * lSpeed * -1;
    }
    is_ask = true;
}

float Jhr_moto::getX()
{
    return posX * 0.001;
}
float Jhr_moto::getY()
{
    return posY * 0.001;
}
uint8 Jhr_moto::getState()
{
    return state;
}
float Jhr_moto::getRadian()
{
    return posRadian;
}

float Jhr_moto::getLinear()
{
    return vLinear * 0.001;
}
float Jhr_moto::getAngular()
{
    return vAngular;
}

/*void show_hex(const char* buf,int len,char*msg){
    char dat[20];
    std::stringstream sstream;
    sstream << "show_hex :" << msg;
    for(int i=0; i < len; i++){
        sprintf(dat,"%02X ",(uint8)buf[i]);
        sstream << dat ;
    }
    std::cout << sstream.str() << std::endl;
}*/

/**
 * @brief 回调函数，计算里程计并发布
 *
 * @param buff_r
 * @param iLen
 */
void Jhr_moto::rcv_callback_1(const char *buff_r, int iLen)
{
    //记录电机的状态值与当前的左右轮脉冲值
    MotoRcv *motoRcv = (MotoRcv *)buff_r;
    if (iLen != 12)
    {
        return;
    }
    static bool isFirst = true;
    unsigned char crc = 0;
    for (int i = 0; i < 11; i++)
    {
        crc += buff_r[i];
    }
    //验证数据头是否错误
    if (crc == (unsigned char)buff_r[11] && motoRcv->head == 0x9bc2)
    {
        //$电机状态标识
        state = motoRcv->state;
        //$ dltL，dltR记录电机的脉冲数只差
        int dltL = motoRcv->lData - lDistance;
        int dltR = rDistance - motoRcv->rData;

        //第一次回调，记录初始的lDistance，rDistance
        if (isFirst)
        {
            //￥ pulse_equivalent表示每个脉冲的弧度
            float fL = abs(dltL + dltR) / pulse_equivalent;
            if (fL < 2000)
                isFirst = false;
            lDistance = motoRcv->lData;
            rDistance = motoRcv->rData;
        }
        ////////////////////////////
        else
        {
            float dL = dltL / pulse_equivalent;
            float dR = dltR / pulse_equivalent;

            if (dL == 0.0f && dR == 0.0f)
            {
                vLinear = 0;
                vAngular = 0;
            }
            else
            {
                vLinear = vLinear0;
                vAngular = vAngular0;
            }
            //￥ 计算机器人的偏航角，wheel_spacing表示轮间距
            float dTh = ((dR - dL) / wheel_spacing);
            dL += dR;
            dL /= 2;
            if (dL < -2000 || dL > 2000) //出现大的跳动属于错误
            {
                std::stringstream sstream;
                sstream << "编码器错误:";
                sstream << "oldL:" << lDistance << '\t';
                sstream << "oldR:" << rDistance << '\t';
                sstream << "newL:" << motoRcv->lData << '\t';
                sstream << "newR:" << motoRcv->rData << '\t';
                sstream << "\r\n";
                jhr_file_record("/home/jhr/1.txt", sstream.str()); //将错误信息保存到文件中
            }
            else
            {
                //% 计算车辆的位置信息
                posX += (dL * cosf(posRadian));
                posY += (dL * sinf(posRadian));
                posRadian += dTh;

                while (posRadian > 3.1416)
                    posRadian -= 6.2832;
                while (posRadian < -3.1416)
                    posRadian += 6.2832;
                odom_publish();
            }
            //将当前的脉冲数保存到系统中，进行下一次的比对
            lDistance = motoRcv->lData;
            rDistance = motoRcv->rData;
        }
        is_ask = true;
    }
}

/**
 * @brief 电机控制回调函数，将控制参数通过udp发送至下位机
 *
 */
void Jhr_moto::moto_loop(void)
{
    static unsigned char buff_s[] = {0xc2, 0x9a, 0x01, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0xff};
    static unsigned char buff_r[100];
    static int iCnt = 0;
    if ((!is_ask) && (iCnt++ < 10))
    {
        if (iCnt == 3)
        {
            std::cout << "通讯失败 ：" << std::endl;
        }
        return;
    }
    is_ask = false;
    iCnt = 0;
    LineMode_handle();//直线句柄
    TurnMode_handle();//旋转句柄
    if (cmd == 3 && state == 0)
    {
        cmd = cmd_1;
    }
    if (cmd == 3 && state != 0)
    {
        buff_s[3] = 3;
        buff_s[7] = 0;
        buff_s[8] = 0;
        buff_s[9] = 0;
        buff_s[10] = 0;
    }
    //给下位机发送停止指令
    else if (lSpeed == 0 && rSpeed == 0)
    {
        if (cmd == 1)
        {
            buff_s[3] = 1;
        }
        else
        {
            buff_s[3] = 2;
        }
        buff_s[7] = 0;
        buff_s[8] = 0;
        buff_s[9] = 0;
        buff_s[10] = 0;
    }
    //给下位机发送速度，kp，ki，ac
    else
    {
        short lS1 = lSpeed * pulse_equivalent;
        short rS1 = rSpeed * pulse_equivalent;
        buff_s[3] = 0;
        buff_s[4] = kp;
        buff_s[5] = ki;
        buff_s[6] = ac;
        buff_s[7] = lS1 & 0xff;
        buff_s[8] = lS1 >> 8;
        buff_s[9] = rS1 & 0xff;
        buff_s[10] = rS1 >> 8;
    }
    unsigned char crc = 0;
    for (int i = 0; i < 11; i++)
    {
        crc += buff_s[i];
    }
    buff_s[11] = crc;
    //通过udp将指令下发至下位机
    Jhr_udp::jhr_udp_arr[2]->send_data((const char *)buff_s, 12);
}

/**
 * @brief cmd_vel回调函数，负责控制线速度，角速度。
 *
 * @param geometry_msgs::Twist tw
 */
void Jhr_moto::tw_callback(const geometry_msgs::Twist &tw)
{
    float linear = tw.linear.x;
    float angular = tw.angular.z;
    int iType = tw.linear.y; //默认是通过速度模式运行
    switch (iType)
    {
    case 30: //原地旋转指定角度  tw.angular.y
        Jhr_moto::setTurnMode(tw.angular.y);
        break;
    case 31: //前进/后退指定距离  tw.angular.y
        Jhr_moto::setLineMode(tw.angular.y);
        break;
    default: //%速度运行模式
        Jhr_moto::setSpeedRos(linear, angular);
        break;
    }
}

/**
 * @brief 里程计发布
 *
 */
void Jhr_moto::odom_publish(void)
{
    if (odom_pub_ptr == NULL)
        return;
    // static int iCnt = 0;
    //  const float covariance[36] = {0.01, 0,      0,      0,      0,      0,
    //                               0,     0.01,   0,      0,      0,      0,
    //                               0,     0,      999999, 0,      0,      0,
    //                               0,     0,      0,      999999, 0,      0,
    //                               0,     0,      0,      0,      999999, 0,
    //                               0,     0,      0,      0,      0,      0.01};
    float x = Jhr_moto::getX();       //获取坐标 X
    float y = Jhr_moto::getY();       //获取坐标 Y
    float th = Jhr_moto::getRadian(); //获取弧度值
    curr_time = ros::Time::now();

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    // //发布tf转化
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = curr_time;
    // odom_trans.header.frame_id = "wheel_odom_link";
    // odom_trans.child_frame_id = "base_link";

    // odom_trans.transform.translation.x = x;
    // odom_trans.transform.translation.y = y;
    // odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation = odom_quat;
    // odom_broadcaster->sendTransform(odom_trans);//发布tf转换。

    // 发布里程机wheel_odom
    nav_msgs::Odometry odom;
    odom.header.stamp = curr_time;
    odom.header.frame_id = "wheel_odom_link";
    for (int i = 0; i < 36; i += 7)
    {
        odom.pose.covariance[i] = 1e-1; // covariance[i] ;;
    }
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = Jhr_moto::getLinear(); //获取线速度
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = Jhr_moto::getAngular(); //获取角速度
    odom_pub_ptr->publish(odom);

    // std::cout <<" th:" <<(int)(th*180/3.14)<<"  x:" <<(float)(th*180/3.14) <<"  y:" <<(float)(th*180/3.14) <<"\r         "<< std::flush;
}