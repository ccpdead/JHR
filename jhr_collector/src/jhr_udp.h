#ifndef __JHR_UDP__
#define __JHR_UDP__
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <poll.h>
#include <iostream>
class Jhr_udp
{
public:
    static struct pollfd udp_poll_s[];
    static void udp_poll_init(const char*ip_addr);
    Jhr_udp(const char* ip,int port);
    ~Jhr_udp();
    void send_data(const char*buf,int len);
    int get_sockfd();
    static Jhr_udp *jhr_udp_arr[];
    void set_rcv_callback(void(*callback)(const char*,int len));
private:
    int sockfd;
    void(*rcv_callback)(const char*,int len);
    struct  sockaddr_in servaddr;
    static pthread_t thread_id;
    static void* thread_start(void*);
};


#endif