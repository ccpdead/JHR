#include "jhr_moto_hls.h"
#include "jhr_moto.h"
#include "jhr_udp.h"
#include <time.h>

int Jhr_moto_hls::rcv_pack_cnt = 0; //记录回调次数
short Jhr_moto_hls::lSpeed_r = 0;
short Jhr_moto_hls::rSpeed_r = 0;
short Jhr_moto_hls::lCode_r = 0;
short Jhr_moto_hls::rCode_r = 0;

/**
 * @brief 将地址进行大小端调换
 *
 * @param dat
 * @return short
 */
short chg_big_small(const char *dat)
{
    char ret[2];
    ret[0] = dat[1];
    ret[1] = dat[0];
    return *(short *)ret;
}

/**
 * @brief x，y，dth坐标计算
 *
 * @param lCode_r
 * @param rCode_r
 */
void Jhr_moto_hls::calcOdom(void)
{
    static short old_l = 0;
    static short old_r = 0;
    short d_l = lCode_r - old_l;
    short d_r = old_r - rCode_r;
    if (d_l >= -2000 && d_l <= 2000 && d_r >= -2000 && d_r <= 2000)
    {
        float dL = d_l / Jhr_moto::pulse_equivalent; //$电机每毫米脉冲数
        float dR = d_r / Jhr_moto::pulse_equivalent;
        float dTh = ((dR - dL) * 1.0f / Jhr_moto::wheel_spacing); //￥电机轮间距
        dL += dR;
        dL /= 2;
        Jhr_moto::posX += (dL * cosf(Jhr_moto::posRadian));
        Jhr_moto::posY += (dL * sinf(Jhr_moto::posRadian));
        Jhr_moto::posRadian += dTh;
        while (Jhr_moto::posRadian > 3.1416)
            Jhr_moto::posRadian -= 6.2832;
        while (Jhr_moto::posRadian < -3.1416)
            Jhr_moto::posRadian += 6.2832;
    }
    else //错误显示
    {
        std::cout << "\n calcOdomErr:" << d_l << "  " << d_r << std::endl;
    }
    old_l = lCode_r;
    old_r = rCode_r;
}

/**
 * @brief 根据从UDP中获得的数据解析出下位机返回的线速度和角速度
 *
 * @param buff_r
 * @param iLen
 */
void Jhr_moto_hls::rcv_callback_1(const char *buff_r, int iLen)
{
    if (iLen < 6)
        return;
    rcv_pack_cnt++;
    unsigned short crc16 = generateCRC16((unsigned char *)buff_r, iLen - 2);
    const char *szCrc = &buff_r[iLen - 2];
    if (*(unsigned short *)szCrc == crc16) //校验？？
    {
        unsigned int iAddr = *(unsigned int *)(&buff_r[2]);
        if (buff_r[1] == 0x43) //读指令返回值
        {                      //读取数据
            switch (iAddr)
            {
            case 0x00510050: //读电机转速
                // std::cout << "读电机转速" << std::endl;
                lSpeed_r = chg_big_small(&buff_r[6]);
                rSpeed_r = chg_big_small(&buff_r[8]);
                Jhr_moto::vLinear = (lSpeed_r - rSpeed_r) * 50.0f / Jhr_moto::pulse_equivalent;
                // printf("vLinear = %f\r\n",Jhr_moto::vLinear);
                // if(lSpeed_r == rSpeed_r){
                //     Jhr_moto::vAngular = lSpeed_r * -0.200f / Jhr_moto::pulse_equivalent / Jhr_moto::wheel_spacing;
                //     printf("lSpeed_r = %d, rSpeed_r = %d\r\n",lSpeed_r, rSpeed_r);
                //     printf("vAangular = %f\r\n",Jhr_moto::vAngular * 10);
                // }else{
                Jhr_moto::vAngular = (lSpeed_r + rSpeed_r) / Jhr_moto::pulse_equivalent / Jhr_moto::wheel_spacing * -100;
                // printf("lSpeed_r = %d, rSpeed_r = %d\r\n",lSpeed_r, rSpeed_r);
                // printf("vAangular = %f\r\n",Jhr_moto::vAngular * 10);
                // }
                break;
            case 0x15511550: //读取编码器
                // std::cout << "读编码器" << std::endl;
                lCode_r = chg_big_small(&buff_r[6]);
                rCode_r = chg_big_small(&buff_r[8]);
                calcOdom(); //里程计估计

                Jhr_moto::odom_publish(); // 在这里调用Jhr_moto::odom_publish（）发布里程计数据
                break;
            }
        }
        else if (buff_r[1] == 0x44) //写指令返回值
        {                           //设置数据
            switch (iAddr)
            {
            case 0x00310021:;
                break; //设置马达转速
            case 0x18331823:;
                break; //设置马达使能
            }
        }
    }
    else
    {   static int i = 0;

        time_t timep;
        time(&timep);
        char tmp[64];
        strftime(tmp,sizeof(tmp)," <%H:%M:%S>",localtime(&timep));
        std::cout << "CRC错误:No."<<i++ <<tmp<< std::endl;
    }
    moto_loop_pri();
}

/**
 * @brief 电机回调函数，在main中30hz频率调用。
 */
void Jhr_moto_hls::moto_loop(void)
{
    static int iCnt = 0;
    //$ 在这里调用moto中的直线模式和旋转模式两个函数
    Jhr_moto::LineMode_handle();
    Jhr_moto::TurnMode_handle();

    if (rcv_pack_cnt == iCnt)
    {
        moto_loop_pri();
        // std::cout << "\n Jhr_moto_hls::rcv_pack_cnt：" << iCnt << "       \r\n";
    }
    else
    {
        iCnt = rcv_pack_cnt;
    }
}

/**
 * @brief 和利时电机驱动部分，将得到的线速度角速度通过udp发送至下位机
 *
 */
void Jhr_moto_hls::moto_loop_pri(void)
{
    static int iCnt = 0;
    //￥ 在switch语句中首先判断电机转速
    switch (iCnt)
    {
    case 0:
        generateEncoder(1);
        break;
    case 1:
        generateSpeed(1);
        break;
    case 2:
        if (Jhr_moto::lSpeed != 0 || Jhr_moto::rSpeed != 0)
            generateEnable(1, 1);
        else
            generateEnable(1, 0); //当目标速度为0时，关闭电机
        break;
    //￥ 通过左右轮速度模式控制电机
    default:
        generateSpeed(1, Jhr_moto::lSpeed, Jhr_moto::rSpeed);
        break;
    }
    iCnt++;
    if (iCnt > 3)
        iCnt = 0;
}

/**
 * @brief 设置马达使能/失能
 * @param addr
 * @param enable  0：失能   1：使能
 * @return
 */
void Jhr_moto_hls::generateEnable(unsigned char addr, unsigned char enable)
{
    char buff[12] = {0x01, 0x44, 0x21, 0x00, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00};
    buff[0] = addr;
    buff[7] = enable;
    buff[9] = enable;
    unsigned short crc16 = generateCRC16((unsigned char *)buff, sizeof(buff) - 2);
    char *szCrc = &buff[sizeof(buff) - 2];
    *(unsigned short *)szCrc = crc16;
    udp_send_data((const char *)buff, sizeof(buff));
}

/**
 * @brief 读取左右电机的速度
 * @param addr
 **/
void Jhr_moto_hls::generateSpeed(unsigned char addr)
{
    char buff[8] = {0x01, 0x43, 0x50, 0x00, 0x51, 0x00};
    buff[0] = addr;
    unsigned short crc16 = generateCRC16((unsigned char *)buff, sizeof(buff) - 2);
    char *szCrc = &buff[sizeof(buff) - 2];
    *(unsigned short *)szCrc = crc16;
    udp_send_data((const char *)buff, sizeof(buff));
}

/**
 * @brief 设置左右电机速度
 * @param addr
 * @param speedL
 * @param speedR
 * @return
 */
void Jhr_moto_hls::generateSpeed(unsigned char addr, short speedL, short speedR)
{
#define CON_MIN_SPEED 30
    short addSP = speedL - speedR;
    if (addSP < 0)
        addSP *= -1;

    if (addSP < 10 && speedL != 0 && Jhr_moto::work_mode == 0)
    {
        if (speedL > -CON_MIN_SPEED && speedL < CON_MIN_SPEED)
        {
            if (speedL < 0)
            {
                speedL = -CON_MIN_SPEED;
                speedR = -CON_MIN_SPEED;
            }
            else
            {
                speedL = CON_MIN_SPEED;
                speedR = CON_MIN_SPEED;
            }
        }
    }

    speedL /= 10;
    speedR /= 10;
    char buff[12] = {0x01, 0x44, 0x23, 0x18, 0x33, 0x18};
    //设置左右轮的速度
    buff[0] = addr;
    buff[6] = speedL >> 8;
    buff[7] = speedL & 0xff;
    buff[8] = speedR >> 8;
    buff[9] = speedR & 0xff;
    unsigned short crc16 = generateCRC16((unsigned char *)buff, sizeof(buff) - 2);
    char *szCrc = &buff[sizeof(buff) - 2];
    *(unsigned short *)szCrc = crc16;
    udp_send_data((const char *)buff, sizeof(buff));
}

/**
 * @brief 获取电机编码器的值
 * @param addr
 * @return
 */
void Jhr_moto_hls::generateEncoder(unsigned char addr)
{
    char buff[8] = {0x01, 0x43, 0x50, 0x15, 0x51, 0x15, 0x00, 0x00};
    buff[0] = addr;
    unsigned short crc16 = generateCRC16((unsigned char *)buff, sizeof(buff) - 2);
    char *szCrc = &buff[sizeof(buff) - 2];
    *(unsigned short *)szCrc = crc16;
    udp_send_data((const char *)buff, sizeof(buff));
}

/**
 * @brief 调用jhr_udp::jhr_udp的send_data来发布数据
 *
 * @param buff
 * @param len
 */
void Jhr_moto_hls::udp_send_data(const char *buff, int len)
{
    //$ 注意，这里的jhr_udp_arr[2]是指向和利时驱动器的ip与端口号
    Jhr_udp::jhr_udp_arr[2]->send_data(buff, len);
}

// ********** modbos *********
/* 高位字节的CRC  值  */
const unsigned char auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};

/* 低位字节的CRC  值  */
const unsigned char auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2,
    0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
    0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE,
    0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
    0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62,
    0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
    0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76,
    0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A,
    0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86,
    0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40};

/**
 * @brief CRC16位校验
 *
 * @param puchMsg
 * @param usDataLen
 * @return unsigned short
 */
unsigned short Jhr_moto_hls::generateCRC16(unsigned char *puchMsg, unsigned short usDataLen)
{
    unsigned char uchCRCHi = 0xFF; // CRC  的高字节初始化
    unsigned char uchCRCLo = 0xFF; // CRC  的低字节初始化
    unsigned char uIndex;          // CRC 查询表索引

    while (usDataLen--) // 完成整个报文缓冲区
    {
        uIndex = uchCRCLo ^ *puchMsg++; //计算  CRC
        uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
        uchCRCHi = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}
