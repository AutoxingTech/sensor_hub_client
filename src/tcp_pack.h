#pragma once
#include <stdint.h>
#include "packet_parser.h"

struct __attribute__((packed)) MsgPack
{
    uint8_t header[2];
    uint32_t length;
    uint16_t crc;
    uint8_t payload[0]; // msg
};

class MsgPackParser : public Parser
{
public:
    MsgPackParser(std::vector<uint8_t> header) : m_header(header) {}

    const std::vector<uint8_t> &header() override { return m_header; }

    ParserResult feed(const uint8_t *bytes, size_t n, size_t *bytesUsed) override;

    const std::vector<uint8_t> m_header;
};
