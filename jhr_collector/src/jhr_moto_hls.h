#ifndef __JHR_MOTO_HLS__
#define __JHR_MOTO_HLS__
#include <iostream>
#include     <stdio.h>
#include     <stdlib.h>

class Jhr_moto_hls
{
public:
    static void moto_loop(void);
    static void rcv_callback_1(const char*buff_r,int iLen);
private:
    static void moto_loop_pri(void);
    static unsigned short generateCRC16 (unsigned char* puchMsg,unsigned short usDataLen);
    static void udp_send_data(const char *buf,int len);
    static void generateEnable(unsigned char addr,unsigned char enable);//设置马达使能/失能
    static void generateSpeed(unsigned char addr);  //读取马达转速 数据高字节在前
    static void generateSpeed(unsigned char addr,short speedL,short speedR); //设置左右马达速度运行 数据高字节在前
    static void generateEncoder(unsigned char addr);    //读取编码器的值，数据高字节在前
    static void calcOdom(void); //计算pos
    static int rcv_pack_cnt;
    static short lSpeed_r;
    static short rSpeed_r;
    static short lCode_r;
    static short rCode_r;
};



#endif