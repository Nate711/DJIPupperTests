#pragma once
#include "Arduino.h"

enum class ParseResultFlag
{
    kError,
    kNothingToRead,
    kReading,
    kDone
};

struct ParseResult
{
    ParseResultFlag result = ParseResultFlag::kError;
    uint32_t message_size;
};

template <uint32_t kBufferSize = 128>
class NonBlockingSerialBuffer
{
private:
    enum class ParserState
    {
        kReading,
        kWaitingForStateCharacter,
    };
    uint8_t start_byte_;
    uint8_t stop_byte_;
    uint32_t write_index_ = 0;

    Stream *stream_;
    bool string_termination_;

    ParserState state_ = ParserState::kWaitingForStateCharacter;

public:
    char buffer_[kBufferSize];
    NonBlockingSerialBuffer(uint8_t start_byte, uint8_t end_byte, Stream &stream, bool string_termination = false);
    ParseResult Feed();
    void FlushStream();
};

template <uint32_t kBufferSize>
NonBlockingSerialBuffer<kBufferSize>::NonBlockingSerialBuffer(uint8_t start_byte, uint8_t stop_byte, Stream &stream, bool string_termination) : start_byte_(start_byte), stop_byte_(stop_byte), stream_(&stream), string_termination_(string_termination) {}

template <uint32_t kBufferSize>
ParseResult NonBlockingSerialBuffer<kBufferSize>::Feed()
{
    ParseResult result;
    bool read_bytes = false;
    while (stream_->available())
    {
        read_bytes = true;
        uint8_t in_byte = Serial.read();
        if (state_ == ParserState::kWaitingForStateCharacter)
        {
            if (in_byte == start_byte_)
            {
                state_ = ParserState::kReading;
            }
        }
        else if (state_ == ParserState::kReading)
        {
            if (in_byte == stop_byte_)
            {
                if (string_termination_)
                {
                    buffer_[write_index_] = '\0';
                }

                // TODO: add checksum to the buffer reader or leave all of that to outside code?
                result.result = ParseResultFlag::kDone;
                result.message_size = write_index_;

                state_ = ParserState::kWaitingForStateCharacter;
                write_index_ = 0;
                return result;
            }
            else
            {
                buffer_[write_index_] = in_byte;
                write_index_++;
            }
        }
    }
    result.result = read_bytes ? ParseResultFlag::kReading : ParseResultFlag::kNothingToRead;
    result.message_size = 0;
    return result;
}

template <uint32_t kBufferSize>
void NonBlockingSerialBuffer<kBufferSize>::FlushStream()
{
    while (stream_->available())
    {
        stream_->read();
    }
}