#include "../libs/socket_lib/tcpsocket.h"

unsigned long long hl64ton(unsigned long long host)
{
	unsigned long long   ret = 0;
	unsigned long long   high, low;
	low = host & 0xFFFFFFFF;
	high = (host >> 32) & 0xFFFFFFFF;
	low = htonl(low);
	high = htonl(high);
	ret = low;
	ret <<= 32;
	ret |= high;
	return   ret;
}

unsigned long long ntohl64(unsigned long long host)
{
	unsigned long long   ret = 0;
	unsigned long long   high, low;
	low = host & 0xFFFFFFFF;
	high = (host >> 32) & 0xFFFFFFFF;
	low = ntohl(low);
	high = ntohl(high);
	ret = low;
	ret <<= 32;
	ret |= high;
	return   ret;
}


#ifdef _WIN32_PLATFORM_
WinSocketSystem::WinSocketSystem()
{
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != NO_ERROR)
	{
		exit(-1);
	}
}

WinSocketSystem::~WinSocketSystem()
{
	WSACleanup();
}
#endif

TCPSocket::TCPSocket() : m_sockfd(INVALID_SOCKET) {}
TCPSocket::~TCPSocket()
{
	if (isValid())
#ifdef _LINUX_PLATFORM_
		::close(m_sockfd);
#else
		::closesocket(m_sockfd);
#endif
}

bool TCPSocket::create()
{
    if (isValid())
        return false;

#ifdef _LINUX_PLATFORM_
	int protocol = 0;
#else
	int protocol = IPPROTO_TCP;
#endif

	if ((m_sockfd = ::socket(AF_INET, SOCK_STREAM, protocol)) == SOCKET_ERROR)
        return false;
    return true;
}

bool TCPSocket::bind(unsigned short int port, const char *ip) const
{
    if (!isValid())
        return false;

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
#ifdef _LINUX_PLATFORM_
    if (ip == NULL)
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    else
        addr.sin_addr.s_addr = inet_addr(ip);
#else
    if (ip == NULL)
        addr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
    else
        addr.sin_addr.S_un.S_addr = inet_addr(ip);
#endif
	if (::bind(m_sockfd, (const struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR)
        return false;
    return true;
}
bool TCPSocket::listen(int backlog) const
{
    if (!isValid())
        return false;

	if (::listen(m_sockfd, backlog) == SOCKET_ERROR)
        return false;
    return true;
}
bool TCPSocket::accept(TCPSocket &clientSocket) const
{
    if (!isValid())
        return false;

    clientSocket.m_sockfd = ::accept(this->m_sockfd, NULL, NULL);
    if (clientSocket.m_sockfd == SOCKET_ERROR)
        return false;
    return true;
}

bool TCPSocket::connect(unsigned short int port, const char *ip) const
{
    if (!isValid())
        return false;
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
#ifdef _LINUX_PLATFORM_
    addr.sin_addr.s_addr = inet_addr(ip);
#else
    addr.sin_addr.S_un.S_addr = inet_addr(ip);
#endif
	if (::connect(m_sockfd, (const struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR)
        return false;
    return true;
}

#if defined _LINUX_PLATFORM_
bool TCPSocket::setNonBlocking(bool flag) const
{
    if (!isValid())
        return false;
    int opt = fcntl(m_sockfd, F_GETFL, 0);
    if (opt == -1)
        return false;
    if (flag)
        opt |= O_NONBLOCK;
    else
        opt &= ~O_NONBLOCK;
    if (fcntl(m_sockfd, F_SETFL, opt) == -1)
        return false;
    return true;
}
#else
bool TCPSocket::setNonBlocking(unsigned long flag) const
{
    if (!isValid())
        return false;
    int opt = ioctlsocket(m_sockfd, FIONBIO, (unsigned long *)&flag);
	if (opt == SOCKET_ERROR)
        return false;
    return true;
}
#endif

bool TCPSocket::reuseaddr() const
{
    if (!isValid())
        return false;

#ifdef _LINUX_PLATFORM_
    int on = 1;
	if (setsockopt(m_sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) == SOCKET_ERROR)
        return false;
#else
	int on = 3000;
	if (setsockopt(m_sockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&on, sizeof(on)) == SOCKET_ERROR)
		return false;
#endif
    return true;
}
bool TCPSocket::close()
{
    if (!isValid())
        return false;
#ifdef _LINUX_PLATFORM_
    ::close(m_sockfd);
#else
    ::closesocket(m_sockfd);
#endif
    m_sockfd = INVALID_SOCKET;
    return true;
}

/** client TCP Socket **/
/*
TCPClient::TCPClient(unsigned short int port, const char *ip)
{
if (create() == false)
cout << "tcp client create error" << endl;
if (connect(port, ip) == false)
cout << "tcp client connect error" << endl;
}
*/
TCPClient::TCPClient(unsigned short int port, const char *ip)
{
	if (create() == false)
	{
		cout << "tcp client create error" << endl;
		return;
	}
	while (connect(port, ip) == false)
	{
		cout << "tcp client connect error" << endl;
#ifdef _WIN32_PLATFORM_
		Sleep(100);
#else
        sleep(0.1);
#endif
	}
	cout << "tcp client connect success" << endl;
}
TCPClient::TCPClient() {}
TCPClient::TCPClient(SOCKET clientfd)
{
    m_sockfd = clientfd;
}
TCPClient::~TCPClient() {}

//重新连接
bool TCPClient::reconnect(unsigned short int port, const char* ip)
{
	if (create() == false)
	{
		cout << "tcp client create error" << endl;
		return false;
	}

	//断开重连
	while (connect(port, ip) == false)
	{
		cout << "tcp client reconnect error" << endl;
		//设定延迟
#ifdef _WIN32_PLATFORM_
		Sleep(100);
#else
        sleep(0.1);
#endif
	}
	return true;
}

/** client端特有的send/receive **/
static ssize_t readn(SOCKET fd, void *buf, size_t count);
static ssize_t writen(SOCKET fd, const void *buf, size_t count);

//send
/*
size_t TCPClient::send(std::string &message) const
{
unsigned int msgLen = message.length();
//cout << "message's size: " << message.length() << endl;
char buf[1024];
memset(buf, 0, sizeof(buf));
message.copy(buf, msgLen);
if (writen(m_sockfd, buf, message.length()) == -1)
cout << "tcp client writen error" << endl;
return message.length();
}
*/
size_t TCPClient::send(std::string &message) const
{
	unsigned int msgLen = message.length();
	char* buf = (char*)malloc(sizeof(char) * msgLen);
	memset(buf, 0, sizeof(buf));
	message.copy(buf, msgLen);
	if (writen(m_sockfd, buf, message.length()) == -1)
	{
		cout << "tcp client writen error" << endl;
		free(buf);
		return 0;
	}
	free(buf);
	return message.length();
}

size_t TCPClient::send(std::string &message, unsigned int id, unsigned long long timestamp) const
{
    Packet buf;
    buf.id = htonl(id);
    //buf.timestamp = htonl(timestamp);
	buf.timestamp = hl64ton(timestamp);
    buf.allmsgLen = htonl(message.length());
    //cout << "send:: message.length() is: " << message.length() << endl;
    int all_bytes = message.length();

    //循环发送
    unsigned int index = 0;
    while (all_bytes > 0)
    {
        string sub_str;
        if (all_bytes < sizeof(buf.text))
        {
            sub_str = message.substr(index, all_bytes);
            buf.msgLen = htonl(sub_str.length());
            sub_str.copy(buf.text, sub_str.length());
            all_bytes -= sizeof(buf.text);
        }
        else
        {
            sub_str = message.substr(index, sizeof(buf.text));
            buf.msgLen = htonl(sub_str.length());
            sub_str.copy(buf.text, sizeof(buf.text));
            index += sizeof(buf.text);
            all_bytes -= sizeof(buf.text);
            //cout << "all_bytes: " << all_bytes << endl;
        }
		if (writen(m_sockfd, &buf, sizeof(buf.msgLen) + sizeof(buf.allmsgLen) + sizeof(buf.timestamp) + sizeof(buf.id) + sub_str.length()) == -1)
		{
			cout << "tcp client writen error" << endl;
			return 0;
		}
    }
    return message.length();
}

//receive
size_t TCPClient::receive(std::string& message, size_t count) const
{
	char* buf = (char*)malloc(sizeof(char) * count);
	memset(buf, 0, sizeof(char) * count);
	if (readn(m_sockfd, buf, sizeof(char) * count) == -1)
	{
		cout << "tcp client readn error" << endl;
		free(buf);
		return 0;
	}
	message.assign(buf, sizeof(char) * count);
	free(buf);
	return message.length();
}

size_t TCPClient::receive(std::string &message) const
{
    char buf[1024];
    memset(buf, 0, sizeof(buf));
	if (readn(m_sockfd, buf, sizeof(buf)) == -1)
	{
		cout << "tcp client readn error" << endl;
		return 0;
	}
    message.assign(buf, sizeof(buf));
    return message.length();
}

size_t TCPClient::receive(std::string &message, unsigned int &id, unsigned long long &timestamp) const
{
    //首先读取头部
    Packet buf = {0, 0, 0, 0, 0};
    //循环读取所有数据
    size_t all_bytes = 0;
    while (true)
    {
        //设定子串
        string sub_str;
        //读取本次发送报文长度
        size_t readBytes = readn(m_sockfd, &buf.msgLen, sizeof(buf.msgLen));
		if (readBytes == (size_t)-1)
		{
			cout << "tcp client readn error" << endl;
			return 0;
		}
		else if (readBytes != sizeof(buf.msgLen))
		{
			cout << "peer connect closed" << endl;
			return 0;
		}

        //读取整个报文的长度
        readBytes = readn(m_sockfd, &buf.allmsgLen, sizeof(buf.allmsgLen));
		//cout << "buf.allmsgLen = " << ntohl(buf.allmsgLen) << endl;
		if (readBytes == (size_t)-1)
		{
			cout << "tcp client readn error" << endl;
			return 0;
		}
		else if (readBytes != sizeof(buf.allmsgLen))
		{
			cout << "peer connect closed" << endl;
			return 0;
		}

        readBytes = readn(m_sockfd, &buf.timestamp, sizeof(buf.timestamp));
		if (readBytes == (size_t)-1)
		{
			cout << "tcp client readn error" << endl;
			return 0;
		}
		else if (readBytes != sizeof(buf.timestamp))
		{
			cout << "peer connect closed" << endl;
			return 0;
		}

        //读取对应id
        readBytes = readn(m_sockfd, &buf.id, sizeof(buf.id));
		if (readBytes == (size_t)-1)
		{
			cout << "tcp client readn error" << endl;
			return 0;
		}
		else if (readBytes != sizeof(buf.id))
		{
			cout << "peer connect closed" << endl;
			return 0;
		}

        //读取相关数据
        size_t lenHost = ntohl(buf.msgLen);
        size_t alllenHost = ntohl(buf.allmsgLen);
        id = ntohl(buf.id);
		//!!!缺了字节转换
		//timestamp = ntohl(buf.timestamp);
		timestamp = ntohl64(buf.timestamp);
        readBytes = readn(m_sockfd, buf.text, lenHost);
		if (readBytes == (size_t)-1)
		{
			cout << "tcp client readn error" << endl;
			return 0;
		}
		else if (readBytes != lenHost)
		{
			cout << "peer connect closed" << endl;
			return 0;
		}

        sub_str.assign(buf.text, lenHost);
        message += sub_str;
        all_bytes += lenHost;
        if (alllenHost == all_bytes)
        {
            break;
        }
    }
    return message.length();
}

size_t TCPClient::read(void *buf, size_t count)
{
#ifdef _LINUX_PLATFORM_
	ssize_t readBytes = ::read(m_sockfd, buf, count);
#else
	ssize_t readBytes = ::recv(m_sockfd, (char*)buf, count,0);
#endif
	if (readBytes == -1)
	{
		cout << "tcp client read error" << endl;
		return 0;
	}
    return (size_t)readBytes;
}
void TCPClient::write(const void *buf, size_t count)
{
#ifdef _LINUX_PLATFORM_
    if (::write(m_sockfd, buf, count) == -1)
#else
	if (::send(m_sockfd, (char*)buf, count,0) == -1)
#endif
        cout << "tcp client write error" << endl;
}
size_t TCPClient::write(const char *msg)
{
#ifdef _LINUX_PLATFORM_
	if (::write(m_sockfd, msg, strlen(msg)) == -1)
	{
		cout << "tcp client write error" << endl;
		return 0;
	}	
#else
	if (::send(m_sockfd, msg, strlen(msg), 0) == -1)
	{
		cout << "tcp client write error" << endl;
		return 0;
	}
#endif
    return strlen(msg);
}

/** Server TCP Socket**/
TCPServer::TCPServer(unsigned short int port, const char *ip, int backlog)
{
    if (create() == false)
        cout << "tcp server create error" << endl;
    if (reuseaddr() == false)
        cout << "tcp server reuseaddr error" << endl;
    if (bind(port, ip) == false)
        cout << "tcp server bind error" << endl;
    if (listen(backlog) == false)
        cout << "tcp server listen error" << endl;
}
TCPServer::~TCPServer() {}
void TCPServer::accept(TCPClient &client) const
{
    //显式调用基类TCPSocket的accept
    if (TCPSocket::accept(client) == SOCKET_ERROR)
        cout << "tcp server accept error" << endl;
}
TCPClient TCPServer::accept() const
{
    TCPClient client;
	if (TCPSocket::accept(client) == SOCKET_ERROR)
        cout << "tcp server accept error" << endl;
    return client;
}

/** readn/writen实现部分 **/
static ssize_t readn(SOCKET fd, void *buf, size_t count)
{
    size_t nLeft = count;
    ssize_t nRead = 0;
    char *pBuf = (char *)buf;
    while (nLeft > 0)
    {
#ifdef _LINUX_PLATFORM_
		if ((nRead = read(fd, pBuf, nLeft)) < 0)
#else
		if ((nRead = recv(fd, pBuf, nLeft,0)) < 0)
#endif
        {
            //如果读取操作是被信号打断了, 则说明还可以继续读
            if (errno == EINTR)
                continue;
            //否则就是其他错误
            else
                return -1;
        }
        //读取到末尾
        else if (nRead == 0)
            return count - nLeft;

        //正常读取
        nLeft -= nRead;
        pBuf += nRead;
    }
    return count;
}
static ssize_t writen(SOCKET fd, const void *buf, size_t count)
{
    size_t nLeft = count;
    ssize_t nWritten = 0;
    char *pBuf = (char *)buf;
    while (nLeft > 0)
    {
#ifdef _LINUX_PLATFORM_
        if ((nWritten = write(fd, pBuf, nLeft)) < 0)
#else
		if ((nWritten = send(fd, pBuf, nLeft,0)) < 0)
#endif
        {
            //如果写入操作是被信号打断了, 则说明还可以继续写入
            if (errno == EINTR)
                continue;
            //否则就是其他错误
            else
                return -1;
        }
        //如果 ==0则说明是什么也没写入, 可以继续写
        else if (nWritten == 0)
            continue;

        //正常写入
        nLeft -= nWritten;
        pBuf += nWritten;
    }
    return count;
}
