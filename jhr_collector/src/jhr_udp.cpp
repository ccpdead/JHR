#include <pthread.h>
#include <errno.h>
#include "jhr_udp.h"

struct pollfd Jhr_udp::udp_poll_s[6];
Jhr_udp *Jhr_udp::jhr_udp_arr[6];
pthread_t Jhr_udp::thread_id;
Jhr_udp::Jhr_udp(const char* ip,int port)
{
    char s_test[] = " ";
    memset(&servaddr, '\0', sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(ip);
    servaddr.sin_port = htons(port);
    sockfd = socket(AF_INET , SOCK_DGRAM , 0);
    send_data(s_test,1);
    rcv_callback = NULL;
}
Jhr_udp::~Jhr_udp()
{
    if(sockfd >0)close(sockfd);
}
int Jhr_udp::get_sockfd()
{
    return sockfd; 
}
/**
 * @brief 配置udp的回调函数
 * 
 * @param callback 
 */
void Jhr_udp::set_rcv_callback(void(*callback)(const char*,int len))
{
    rcv_callback = callback;
}
void Jhr_udp::send_data(const char*buf,int len)
{
    sendto(sockfd,buf,len,0,(struct sockaddr*)&servaddr,sizeof(servaddr));
}
void Jhr_udp::udp_poll_init(const char*ip_addr)
{
    for(int i=0;i< sizeof(udp_poll_s)/sizeof(udp_poll_s[0]);i++)
    {
        udp_poll_s[i].fd = -1;
        udp_poll_s[i].events = POLLIN;
    }
    jhr_udp_arr[0] = new Jhr_udp(ip_addr,10001);
    jhr_udp_arr[1] = new Jhr_udp(ip_addr,10006);
    jhr_udp_arr[2] = new Jhr_udp(ip_addr,10007);
    jhr_udp_arr[3] = new Jhr_udp(ip_addr,10008);
    jhr_udp_arr[4] = new Jhr_udp(ip_addr,10009);
    jhr_udp_arr[5] = new Jhr_udp(ip_addr,10010);

    udp_poll_s[0].fd = jhr_udp_arr[0]->get_sockfd();
    udp_poll_s[1].fd = jhr_udp_arr[1]->get_sockfd();
    udp_poll_s[2].fd = jhr_udp_arr[2]->get_sockfd();
    udp_poll_s[3].fd = jhr_udp_arr[3]->get_sockfd();
    udp_poll_s[4].fd = jhr_udp_arr[4]->get_sockfd();
    udp_poll_s[5].fd = jhr_udp_arr[5]->get_sockfd();
    //￥ 多线程udp通信
    int ret = pthread_create(&thread_id,NULL,thread_start,NULL);
    if(ret != 0)
    {
        std::cout<<"pthread_create error :" << ret <<std::endl;
    }
}
void* Jhr_udp::thread_start(void*)
{
    int i_1;
    ssize_t len;
    char rec_buf[2048];
    while(1){
        //通过多线程实现udp通信
        i_1 = poll(udp_poll_s,6,-1);
        if(i_1 < 0){
            printf("Poll error %d '%s'\n", errno, strerror(errno));
        }else if(i_1 == 0){
            printf("Poll timeout\n");
            continue;
        }
        for(int i = 0; i < 6; i ++){
            if(udp_poll_s[i].revents & POLLIN)
            {
                len = recv(udp_poll_s[i].fd, rec_buf, 2048,0);
                if(len > 0){
                    if(jhr_udp_arr[i]->rcv_callback != NULL){
                        jhr_udp_arr[i]->rcv_callback(rec_buf,len);
                    }
                }
            }
        }       
    }
}