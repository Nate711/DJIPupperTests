#pragma once
#include "Arduino.h"

enum class BufferResult
{
    kError,
    kNothingToRead,
    kReading,
    kDone
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

    // Construct the nonblockingserialbuffer object. Parameters include the start_byte (what to look for to start reading a message),
    // a stop byte (what to look for to end reading), a reference to the input stream, and a boolean indicating whether to stick on a '\0' 
    // to the buffer or not when done reading.
    NonBlockingSerialBuffer(uint8_t start_byte, uint8_t stop_byte, Stream &stream, bool string_termination = false);

    // Reads from the serial input buffer, adding anything new to the buffer, and returns a BufferResult - kError, kNothingToRead, kReading, kDone.
    BufferResult Read();

    // Removes all the bytes from the input stream.
    void FlushStream();
};

template <uint32_t kBufferSize>
NonBlockingSerialBuffer<kBufferSize>::NonBlockingSerialBuffer(uint8_t start_byte, uint8_t stop_byte, Stream &stream, bool string_termination) : start_byte_(start_byte), stop_byte_(stop_byte), stream_(&stream), string_termination_(string_termination) {}

template <uint32_t kBufferSize>
BufferResult NonBlockingSerialBuffer<kBufferSize>::Read()
{
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

                state_ = ParserState::kWaitingForStateCharacter;
                write_index_ = 0;
                return BufferResult::kDone;
                ;
            }
            else
            {
                buffer_[write_index_] = in_byte;
                write_index_++;
            }
        }
    }
    return read_bytes ? BufferResult::kReading : BufferResult::kNothingToRead;
}

template <uint32_t kBufferSize>
void NonBlockingSerialBuffer<kBufferSize>::FlushStream()
{
    while (stream_->available())
    {
        stream_->read();
    }
}