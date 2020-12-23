#ifndef __TCPSOCKET_H__
#define __TCPSOCKET_H__
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sstream>
#include "platform.h"
#include "Rte_Type.h"
//#include "VMCParking.h"

#ifdef _LINUX_PLATFORM_
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <exception>
#include <cstdlib>
#include <sys/types.h> 
#include <sys/wait.h>
#include <netdb.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#else
#include <WINSOCK2.H>
#include <stdlib.h>
#include <stddef.h>
#include <BaseTsd.h>
#include <sys/select.h>
typedef SSIZE_T ssize_t;
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#pragma comment(lib, "ws2_32.lib")
#pragma warning(disable:4996)
#endif
#pragma once
using namespace std;

#ifdef _WIN32_PLATFORM_
class WinSocketSystem
{
public:
	WinSocketSystem();
	~WinSocketSystem();
protected:
	WSADATA wsaData;
};
#endif

unsigned long long hl64ton(unsigned long long host);
unsigned long long ntohl64(unsigned long long host);

/*模板-序列化及反序列化*/
template <typename T>
void data_serialization(string& msg, T& data)
{
	char* buf = (char *)malloc(sizeof(char) * sizeof(T));
	memset(buf, 0, sizeof(T));
	memcpy(buf, &data, sizeof(T));
	msg.assign(buf, sizeof(T));
	free(buf);
}

template <typename T>
void data_deserialization(string& msg, T& data)
{
	char* buf = (char *)malloc(sizeof(char) * sizeof(T));
	memset(buf, 0, sizeof(T));
	msg.copy(buf, sizeof(T));
	memcpy(&data, buf, sizeof(T));
	free(buf);
}


class TCPSocket
{
protected:
    TCPSocket();
    virtual ~TCPSocket();

    bool create();
    bool bind(unsigned short int port, const char *ip = NULL) const;
    bool listen(int backlog = SOMAXCONN) const;
    bool accept(TCPSocket &clientSocket) const;
    bool connect(unsigned short int port, const char *ip) const;

    /**注意: TCPSocket基类并没有send/receive方法**/
    bool reuseaddr() const;
    bool isValid() const
    {
        return (m_sockfd != INVALID_SOCKET);
    }

public:
    bool close();
    SOCKET getfd() const
    {
        return m_sockfd;
    }
#ifdef _LINUX_PLATFORM_
    bool setNonBlocking(bool flag) const;//flag: true=SetNonBlock, false=SetBlock
#else
    bool setNonBlocking(unsigned long flag) const;//flag: 1 -- 非阻塞，0 -- 阻塞 
#endif
protected:
    SOCKET m_sockfd;
};

/** TCP Client **/
class TCPClient : public TCPSocket
{
private:
    typedef struct Packet
    {
        unsigned int    msgLen;     //数据部分的长度(网络字节序)
        unsigned int    allmsgLen;
        unsigned long long    timestamp;
        unsigned int    id;
        char      text[1024]; //报文的数据部分
    }Packet;
public:
    TCPClient(unsigned short int port, const char *ip);
    TCPClient();
    TCPClient(SOCKET clientfd);
    ~TCPClient();
 
    //size_t send(const std::string& message) const;
    size_t send(std::string& message, unsigned int id, unsigned long long timetamp) const;
    size_t send(std::string& message) const;
	//size_t send(std::string& message,size_t count) const;
    size_t receive(std::string& message, unsigned int &id, unsigned long long &timestamp) const;
    size_t receive(std::string& message) const;
	size_t receive(std::string& message, size_t count) const;
    size_t read(void *buf, size_t count);
    void   write(const void *buf, size_t count);
    size_t write(const char *msg);
	bool reconnect(unsigned short int port, const char* ip);
};

/** TCP Server **/
class TCPServer : public TCPSocket
{
public:
    TCPServer(unsigned short int port, const char *ip = NULL, int backlog = SOMAXCONN);
    ~TCPServer();
    void accept(TCPClient &client) const;
    TCPClient accept() const;
};

#endif
