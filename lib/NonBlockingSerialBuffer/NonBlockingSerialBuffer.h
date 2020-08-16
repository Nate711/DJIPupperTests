#pragma once
#include "Arduino.h"

enum class BufferResult
{
    kError,
    kNothingToRead,
    kReading,
    kDone
};

template <uint32_t kBufferSize = 256>
class NonBlockingSerialBuffer
{
private:
    enum class ParserState
    {
        kWaitingForStartCharacter,
        kReadingMessageLength,
        kReadingPayload,
    };
    const uint8_t start_byte_;
    uint8_t message_length_;
    uint32_t write_index_ = 0;

    Stream *stream_;
    const bool string_termination_;

    ParserState state_ = ParserState::kWaitingForStartCharacter;

public:
    char buffer_[kBufferSize];

    // Construct the nonblockingserialbuffer object. Parameters include the start_byte (what to look for to start reading a message),
    // a reference to the input stream, and a boolean indicating whether to stick on a '\0'
    // to the buffer or not when done reading.
    NonBlockingSerialBuffer(uint8_t start_byte, Stream &stream, bool string_termination = false);

    // Reads from the serial input buffer, adding anything new to the buffer, and returns a BufferResult - kError, kNothingToRead, kReadingPayload, kDone.
    BufferResult Read();

    // Removes all the bytes from the input stream.
    void FlushStream();
};

template <uint32_t kBufferSize>
NonBlockingSerialBuffer<kBufferSize>::NonBlockingSerialBuffer(uint8_t start_byte, Stream &stream, bool string_termination) : start_byte_(start_byte), stream_(&stream), string_termination_(string_termination) {}

template <uint32_t kBufferSize>
BufferResult NonBlockingSerialBuffer<kBufferSize>::Read()
{
    bool read_bytes = false;
    while (stream_->available())
    {
        read_bytes = true;
        uint8_t in_byte = Serial.read();
        if (state_ == ParserState::kWaitingForStartCharacter)
        {
            if (in_byte == start_byte_)
            {
                state_ = ParserState::kReadingMessageLength;
            }
        }
        else if (state_ == ParserState::kReadingMessageLength)
        {
            message_length_ = in_byte;
            state_ = ParserState::kReadingPayload;
        }
        else if (state_ == ParserState::kReadingPayload)
        {   
            buffer_[write_index_] = in_byte;
            write_index_++;
            
            if (write_index_ == message_length_)
            {
                if (string_termination_)
                {
                    buffer_[write_index_] = '\0';
                }

                // TODO: add checksum to the buffer reader or leave all of that to outside code?
                state_ = ParserState::kWaitingForStartCharacter;
                write_index_ = 0;
                return BufferResult::kDone;
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