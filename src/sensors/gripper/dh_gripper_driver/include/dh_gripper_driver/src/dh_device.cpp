#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

#include "dh_device.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>


int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

int serial_connect(std::string portname, int Baudrate )
{
    int fd = -1;
    fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname.c_str(), strerror(errno));
        return -1;
    }

    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    if(Baudrate == 115200)
    {
        set_interface_attribs(fd, B115200);
    }
    else if(Baudrate == 38400)
    {
        set_interface_attribs(fd, B38400);
    }
    else if(Baudrate == 19200)
    {
        set_interface_attribs(fd, B19200);
    }
    else if(Baudrate == 9600)
    {
        set_interface_attribs(fd, B9600);
    }
    return fd;
}

int tcp_connect(std::string ip_port)
{
    std::string servInetAddr = ip_port.substr(0, ip_port.find(":"));
    int PORT = atoi(ip_port.substr(ip_port.find(":") + 1, ip_port.size() - ip_port.find(":") - 1).c_str());

    /*创建socket*/
    struct sockaddr_in serv_addr;
    int sockfd = -1;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) != -1)
    {
      printf("Socket id = %d \n", sockfd);
      /*设置sockaddr_in 结构体中相关参数*/
      serv_addr.sin_family = AF_INET;
      serv_addr.sin_port = htons(PORT);
      inet_pton(AF_INET, servInetAddr.c_str(), &serv_addr.sin_addr);
      bzero(&(serv_addr.sin_zero), 8);
      /*调用connect 函数主动发起对服务器端的连接*/
      if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
      {
        printf("Connect failed!\n");
        return -1;
      }
      else
      {
        printf("connected\n");
        return sockfd;
      }
    }
    else
    {
       printf("Socket failed!\n");
       return -1;
    }
}

int connect_device(std::string portname, int parameter )
{
    int fd;
    if(portname.find(":")!= portname.npos)
    {
       fd = tcp_connect(portname);
    }
    else
    {
       fd = serial_connect(portname, parameter );
    }
    
    //set_mincount(fd, 0);                /* set to pure timed read */
    return fd;
}

void disconnect_device(int fd)
{
    close(fd);
}


int device_wrire(int fd, char *data, int len)
{
    int wlen;
    wlen = write(fd, data, len);
    if (wlen == len )  
	{  
		//printf("send data is %x\n",data);
		return wlen;  
	}       
    else     
	{  
		tcflush(fd,TCOFLUSH);  
		return 0;  
	} 
}

int device_read(int fd, char *data, int data_len)
{
    // data_len is the expected frame size (Modbus read reply is 7 bytes,
    // write echo is 8). A single read() often returns a short piece; keep
    // collecting until the frame is complete or the 200 ms budget runs out.
    if (data_len <= 0)
        return -1;

    int got = 0;
    struct timeval start;
    gettimeofday(&start, NULL);
    const int budget_us = 200000;

    while (got < data_len)
    {
        struct timeval now;
        gettimeofday(&now, NULL);
        int elapsed = (int)((now.tv_sec - start.tv_sec) * 1000000 + (now.tv_usec - start.tv_usec));
        int left = budget_us - elapsed;
        if (left <= 0)
            break;

        fd_set fs_read;
        FD_ZERO(&fs_read);
        FD_SET(fd, &fs_read);
        struct timeval time;
        time.tv_sec = left / 1000000;
        time.tv_usec = left % 1000000;
        int fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
        if (fs_sel <= 0)
            break;

        int len = read(fd, data + got, data_len - got);
        if (len <= 0)
            break;
        got += len;
    }
    return got > 0 ? got : -1;
}


