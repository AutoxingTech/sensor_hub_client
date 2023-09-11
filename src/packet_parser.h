#pragma once

#include <ros/time.h>
#include <algorithm>
#include <stdint.h>

#define UART_BUFFER_MAX_SIZE 1024 * 1024

enum ParserResult
{
    ParserResult_succ = 0,
    ParserResult_incomplete = 1,
    ParserResult_failed = 2
};

class Parser
{
public:
    virtual ~Parser(){};
    virtual const std::vector<uint8_t> &header() = 0;

    virtual ParserResult feed(const uint8_t *bytes, size_t n, size_t *bytesUsed) = 0;
};

class ParserManagerDelegate
{
public:
    virtual void ParserManager_packetFound(const std::vector<uint8_t> &header, ros::Time time, const uint8_t *pack,
                                           size_t bytes) = 0;
};

/**
demo code:
```
struct __attribute__((packed)) ExifPack
{
    char header[4]; // EXIF
    u8 a;
    u8 b;
    u8 c;
    u8 sum;
};

class ExifParser : public Parser
{
public:
    const char* header() override { return "EXIF"; }

    ParserResult feed(const u8* bytes, size_t n, size_t* bytesUsed) override
    {
        if (n < sizeof(ExifPack))
            return ParserResult_incomplete;

        ExifPack* pack = (ExifPack*)bytes;
        *bytesUsed = sizeof(ExifPack);
        if (pack->a + pack->b + pack->c == pack->sum)
            return ParserResult_succ;
        else
            return ParserResult_failed;
    }
};
```
*/

class ParserManager
{
public:
    ParserManager(ParserManagerDelegate *d) { m_delegate = d; }

    void addParser(Parser *parser) { m_parsers.push_back(parser); }
    void addParsersFromAnotherManager(const ParserManager &r)
    {
        m_parsers.insert(m_parsers.end(), r.m_parsers.begin(), r.m_parsers.end());
    }

    void feed(const uint8_t *bytes, size_t n)
    {
        m_buffer.insert(m_buffer.end(), bytes, bytes + n);

        while (true)
        {
            if (m_currentParser == NULL)
            {
                // find header to determine parser
                int minPos = INT_MAX;
                for (auto parser : m_parsers)
                {
                    int pos = findHeader(parser, &m_buffer[0], m_buffer.size());
                    if (pos != -1)
                    {
                        if (pos < minPos)
                        {
                            m_time = ros::Time::now();
                            minPos = pos;
                            m_currentParser = parser;
                        }
                    }
                }

                if (minPos != INT_MAX) // found
                    m_buffer.erase(m_buffer.begin(), m_buffer.begin() + minPos);
                else
                    return;
            }

            if (m_currentParser != NULL)
            {
                size_t bytesUsed = 0;
                ParserResult result = m_currentParser->feed(&m_buffer[0], m_buffer.size(), &bytesUsed);
                if (result == ParserResult_incomplete)
                    return;
                else if (result == ParserResult_succ)
                    m_delegate->ParserManager_packetFound(m_currentParser->header(), m_time, &m_buffer[0], bytesUsed);

                m_buffer.erase(m_buffer.begin(), m_buffer.begin() + bytesUsed);
                m_currentParser = NULL;
            }
        }

        if (m_buffer.size() > UART_BUFFER_MAX_SIZE)
            m_buffer.clear();
    }

private:
    int findHeader(Parser *parser, const uint8_t *bytes, size_t n)
    {
        if (n < parser->header().size())
            return -1;

        const uint8_t *h = parser->header().data();
        const uint8_t *pos = std::search(bytes, bytes + n, h, h + parser->header().size());
        if (pos == bytes + n)
            return -1;
        else
            return (int)(pos - bytes);
    }

private:
    ParserManagerDelegate *m_delegate;
    Parser *m_currentParser = NULL;
    std::vector<Parser *> m_parsers;
    std::vector<uint8_t> m_buffer;
    ros::Time m_time;
};
