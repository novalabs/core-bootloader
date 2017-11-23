/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

//--- DEBUGGING DEFINES -------------------------------------------------------
/// For production: both must be false ///
#ifndef FORCE_LOADER
#define FORCE_LOADER    false
#endif
#ifndef OVERRIDE_LOADER
#define OVERRIDE_LOADER false
#endif
//-----------------------------------------------------------------------------

#define CORE_PACKED          __attribute__((packed))
#define CORE_PACKED_ALIGNED  __attribute__((aligned(4), packed))

extern THD_WORKING_AREA(bootloaderThreadWorkingArea, 4096);
THD_FUNCTION(bootloaderThread, arg);

#include <cstddef>
#include <core/Array.hpp>

namespace bootloader {

// WARNING: KEEP THE FOLLOWING SYNCHED WITH THE MW
using ProductUID = uint16_t;
using ModuleUID  = uint32_t;
using ModuleType = Array<char, 12>;
using ModuleName = Array<char, 16>;

static const uint32_t SHORT_MESSAGE_LENGTH = 8;
static const uint32_t LONG_MESSAGE_LENGTH  = 48;

static const uint32_t MAXIMUM_MESSAGE_LENGTH = 48;

#define BOOTLOADER_MASTER_TOPIC_NAME "BOOTLOADERMSTR"
#define BOOTLOADER_MASTER_TOPIC_ID ((uint8_t)0xFC)

#define BOOTLOADER_TOPIC_NAME "BOOTLOADER"
#define BOOTLOADER_TOPIC_ID ((uint8_t)0xFD)

enum class MessageType : uint8_t {
    NONE           = 0x00,
    REQUEST        = 0x01,
    IDENTIFY_SLAVE = 0x02,
    SELECT_SLAVE   = 0x10,
    DESELECT_SLAVE = 0x11,

    ERASE_CONFIGURATION      = 0x04,
    ERASE_PROGRAM            = 0x05,
    WRITE_PROGRAM_CRC        = 0x06,
    ERASE_USER_CONFIGURATION = 0x07,

    // MODULE_NAME         = 0x25,
    // READ_MODULE_NAME    = 0x26,
    WRITE_MODULE_NAME   = 0x27,
    WRITE_MODULE_CAN_ID = 0x28,
    DESCRIBE_V1         = 0x29,
    DESCRIBE_V2         = 0x26,
    DESCRIBE_V3         = 0x30,

    IHEX_WRITE = 0x50,
    IHEX_READ  = 0x51,

    RESET            = 0x60,
    BOOTLOAD         = 0x70,
    BOOTLOAD_BY_NAME = 0x71,

    MASTER_ADVERTISE = 0xA0,
    MASTER_IGNORE    = 0xA1,
    MASTER_FORCE     = 0xA2,

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
    Message() : command(MessageType::NONE) {}

    Message(
        MessageType c
    ) :
        command(c)
    {}

    MessageType command;

    const Message*
    asMessage()
    {
        return this;
    }
}

CORE_PACKED_ALIGNED;


template <std::size_t _MESSAGE_LENGTH>
struct MessageBase:
    public Message {
    static const std::size_t MESSAGE_LENGTH = _MESSAGE_LENGTH;

    MessageBase() : Message(MessageType::NONE), sequenceId(0) {}

    MessageBase(
        MessageType c
    ) : Message(c), sequenceId(0)
    {}

    uint8_t sequenceId;

    void
    copyTo(
        uint8_t* buffer
    ) const
    {
        const uint8_t* f = reinterpret_cast<const uint8_t*>(this);
        uint8_t*       t = buffer;

        for (std::size_t i = 0; i < _MESSAGE_LENGTH; i++) {
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

        for (std::size_t i = 0; i < _MESSAGE_LENGTH; i++) {
            *(t++) = *(f++);
        }
    }
}

CORE_PACKED_ALIGNED;

template <typename _CONTAINER, MessageType _TYPE, typename _PAYLOAD>
struct Message_:
    public _CONTAINER {
    //static const std::size_t MESSAGE_LENGTH = _CONTAINER::MESSAGE_LENGTH;
    using ContainerType = _CONTAINER;
    using PayloadType   = _PAYLOAD;

    Message_() :
        ContainerType(_TYPE)
    {}

    PayloadType data;

    uint8_t padding[ContainerType::MESSAGE_LENGTH - sizeof(ContainerType) - sizeof(data)];
}

CORE_PACKED_ALIGNED;

template <typename CONTAINER>
class AcknowledgeMessage:
    public CONTAINER
{
public:
    using ContainerType = CONTAINER;

    AcknowledgeMessage() : CONTAINER(MessageType::ACK), status(AcknowledgeStatus::NONE), type(MessageType::NONE) {}

    AcknowledgeStatus status;
    MessageType       type;
}

CORE_PACKED_ALIGNED;

template <typename CONTAINER, typename PAYLOAD>
class AcknowledgeMessage_:
    public AcknowledgeMessage<CONTAINER>
{
public:
    using ContainerType = CONTAINER;
    using PayloadType   = PAYLOAD;

    PAYLOAD data;

    uint8_t padding[ContainerType::MESSAGE_LENGTH - sizeof(AcknowledgeMessage<CONTAINER>) - sizeof(data)];
}

CORE_PACKED_ALIGNED;

using ShortMessage = MessageBase<8>;
using LongMessage  = MessageBase<48>;
}
