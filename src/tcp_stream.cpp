#include "tcp_stream.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>

bool TcpStream::open(std::string ip, int port = 8091)
{
    // 创建 socket
    m_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_sockfd == -1)
    {
        return false;
    }

    // 设置服务器地址和端口
    struct sockaddr_in server_addr
    {
    };
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &(server_addr.sin_addr)) <= 0)
    {
        return false;
    }

    // 连接服务器
    if (connect(m_sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        return false;
    }

    // 设置套接字非阻塞模式
    int flags = fcntl(m_sockfd, F_GETFL, 0);
    fcntl(m_sockfd, F_SETFL, flags | O_NONBLOCK);

    m_connected = true;
    return true;
}

bool TcpStream::close()
{
    if (m_sockfd != -1)
    {
        ::close(m_sockfd);
        m_sockfd = -1;
        m_connected = false;
        return true;
    }
    return false;
}

bool TcpStream::isConnected()
{
    return m_connected;
}

size_t TcpStream::read(uint8_t *buffer, size_t size)
{
    ssize_t num_bytes = recv(m_sockfd, buffer, size, 0);
    if (num_bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
        m_connected = false;
        return 0;
    }
    return static_cast<size_t>(num_bytes);
}

size_t TcpStream::write(const uint8_t *buffer, size_t size)
{
    ssize_t num_bytes = ::send(m_sockfd, buffer, size, MSG_NOSIGNAL);
    if (num_bytes < 0)
    {
        m_connected = false;
        return 0;
    }
    return static_cast<size_t>(num_bytes);
}