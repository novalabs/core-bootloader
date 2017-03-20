/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#define CORE_PACKED          __attribute__((packed))
#define CORE_PACKED_ALIGNED  __attribute__((aligned(4), packed))

extern THD_WORKING_AREA(bootloaderThreadWorkingArea, 4096);
THD_FUNCTION(bootloaderThread, arg);

#include <cstddef>
#include <core/Array.hpp>

namespace bootloader {
using ModuleType = Array<char, 12>;
using ModuleName = Array<char, 14>;

static const uint32_t MESSAGE_LENGTH = 48;

enum class MessageType : uint8_t {
    NONE           = 0x00,
    REQUEST        = 0x01, //
    IDENTIFY_SLAVE = 0x02, //
    SELECT_SLAVE   = 0x10, //
    DESELECT_SLAVE = 0x11, //

    ERASE_CONFIGURATION      = 0x04,
    ERASE_PROGRAM            = 0x05,
    WRITE_PROGRAM_CRC        = 0x06,
    ERASE_USER_CONFIGURATION = 0x07,

    MODULE_NAME       = 0x25,
    READ_MODULE_NAME  = 0x26,
    WRITE_MODULE_NAME = 0x27,

    IHEX_WRITE = 0x50,
    IHEX_READ  = 0x51,

    RESET            = 0x60,
    BOOTLOAD         = 0x70,
    BOOTLOAD_BY_NAME = 0x71,

    ACK = 0xFF
};

enum class AcknowledgeStatus : uint8_t {
    NONE            = 0x00,
    OK              = 0x01,
    WRONG_UID       = 0x02,
    WRONG_SEQUENCE  = 0x03,
    DISCARD         = 0x04,
    NOT_SELECTED    = 0x05,
    NOT_IMPLEMENTED = 0x06,
    BROKEN          = 0x07,
    ERROR           = 0x08,
    IHEX_OK         = 0x09,
    DO_NOT_ACK      = 0x0A,
};

struct Message {
    Message() : command(MessageType::NONE), sequenceId(0) {}

    Message(
        MessageType c
    ) :
        command(c),
        sequenceId(0)
    {}

    MessageType command;
    uint8_t     sequenceId;

    void
    copyTo(
        uint8_t* buffer
    ) const
    {
        const uint8_t* f = reinterpret_cast<const uint8_t*>(this);
        uint8_t*       t = buffer;

        for (std::size_t i = 0; i < MESSAGE_LENGTH; i++) {
            *(t++) = *(f++);
        }
    }

    void
    copyFrom(
        const uint8_t* buffer
    )
    {
        const uint8_t* f = buffer;
        uint8_t*       t = reinterpret_cast<uint8_t*>(this);

        for (std::size_t i = 0; i < MESSAGE_LENGTH; i++) {
            *(t++) = *(f++);
        }
    }

    const Message*
    asMessage()
    {
        return this;
    }
}

CORE_PACKED_ALIGNED;

template <MessageType TYPE, typename PAYLOAD>
struct Message_:
    Message {
    using PayloadType = PAYLOAD;
    Message_() :
        Message(TYPE)
    {}

    PAYLOAD data;

    uint8_t padding[MESSAGE_LENGTH - sizeof(Message) - sizeof(data)];
#if 0
    static const PAYLOAD&
    getDataFromBuffer(
        const uint8_t* buffer
    )
    {
        return reinterpret_cast<const Message_<TYPE, PAYLOAD>*>(buffer)->data;
    }

    static void
    setDataToBuffer(
        const PAYLOAD& payload,
        uint8_t*       buffer
    )
    {
        const uint8_t* f = reinterpret_cast<const uint8_t*>(&payload);
        uint8_t*       t = buffer + sizeof(Message);

        for (std::size_t i = 0; i < sizeof(PAYLOAD); i++) {
            *(t++) = *(f++);
        }
    }
#endif // if 0
}

CORE_PACKED_ALIGNED;

class AcknowledgeMessage:
    public Message
{
public:
    AcknowledgeMessage() : Message(MessageType::ACK), status(AcknowledgeStatus::NONE), type(MessageType::NONE) {}

    AcknowledgeStatus status;
    MessageType       type;
}

CORE_PACKED_ALIGNED;

template <typename PAYLOAD>
class AcknowledgeMessage_:
    public AcknowledgeMessage
{
public:
    PAYLOAD data;

    uint8_t padding[MESSAGE_LENGTH - sizeof(Message) - sizeof(data)];
}

CORE_PACKED_ALIGNED;
}
