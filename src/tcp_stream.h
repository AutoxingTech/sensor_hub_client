#pragma once
#include <cstdio>
#include <stdint.h>
#include <string>

class TcpStream
{
public:
    bool open(std::string ip, int port);
    bool close();
    bool isConnected();

    int read(uint8_t *buffer, size_t size);
    int write(const uint8_t *buffer, size_t size);

private:
    bool m_connected;
    int m_sockfd;
};