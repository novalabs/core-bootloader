/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/bootloader/bootloader.hpp>
#include <core/bootloader/hw/hw_utils.hpp> // UID

namespace bootloader {
namespace payload {
struct EMPTY {};

struct NAME {
    ModuleName name;
};

struct UID {
    hw::UID uid;
};

struct UIDAndMaster {
    hw::UID uid;
    uint8_t masterID;
};

struct UIDAndCRC {
    hw::UID  uid;
    uint32_t crc;
};

struct UIDAndName {
    hw::UID    uid;
    ModuleName name;
};

struct UIDAndAddress {
    hw::UID  uid;
    uint32_t address;
};

struct IHex {
    enum Type : uint8_t {
        BEGIN = 0x01,
        DATA  = 0x02,
        END   = 0x03
    };

    using Data = char[44];

    Type type;
    Data string;
};

struct Announce {
    hw::UID    uid;
    uint16_t   userFlashSize;
    uint16_t   programFlashSize;
    ModuleType moduleType;
    ModuleName moduleName;
};
}

namespace messages {
// SLAVE -> MASTER
using Announce = Message_<MessageType::REQUEST, payload::Announce>;

// MASTER -> SLAVE
using Bootload       = Message_<MessageType::BOOTLOAD, payload::EMPTY>;
using BootloadByName = Message_<MessageType::BOOTLOAD_BY_NAME, payload::NAME>;

using IdentifySlave = Message_<MessageType::IDENTIFY_SLAVE, payload::UID>;
using SelectSlave   = Message_<MessageType::SELECT_SLAVE, payload::UIDAndMaster>;
using DeselectSlave = Message_<MessageType::DESELECT_SLAVE, payload::UID>;

using EraseConfiguration = Message_<MessageType::ERASE_CONFIGURATION, payload::UID>;
using EraseProgram       = Message_<MessageType::ERASE_PROGRAM, payload::UID>;
using WriteProgramCrc    = Message_<MessageType::WRITE_PROGRAM_CRC, payload::UIDAndCRC>;

using IHexData = Message_<MessageType::IHEX_READ, payload::IHex>;

using IHexRead = Message_<MessageType::IHEX_WRITE, payload::UIDAndAddress>;

using Reset = Message_<MessageType::RESET, payload::UID>;

using ReadName = Message_<MessageType::READ_MODULE_NAME, payload::UID>;
using WriteModuleName = Message_<MessageType::WRITE_MODULE_NAME, payload::UIDAndName>;
}

class AcknowledgeUID:
    public AcknowledgeMessage_<payload::UID>
{
public:
    AcknowledgeUID(
        uint8_t           sequence,
        const Message*    message,
        AcknowledgeStatus status
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message->command);
        this->data.uid   = hw::getUID();
        this->status     = status;
    }

    AcknowledgeUID(
        uint8_t           sequence,
        const Message&    message,
        AcknowledgeStatus status
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message.command);
        this->data.uid   = hw::getUID();
        this->status     = status;
    }
}

CORE_PACKED_ALIGNED;

class AcknowledgeString:
    public AcknowledgeMessage_<char[44]>
{
public:
    AcknowledgeString(
        uint8_t           sequence,
        const Message*    message,
        AcknowledgeStatus status,
        const char*       string,
        size_t&           offset
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message->command);
        std::size_t i   = offset;
        std::size_t cnt = 0;

        if (status == AcknowledgeStatus::OK) {
            while (string[i] != 0 && cnt < sizeof(this->data)) {
                this->data[cnt] = string[i];
                i++;
                cnt++;
            }

            if (cnt < sizeof(this->data)) {
                while (cnt < sizeof(this->data)) {
                    this->data[cnt] = 0;
                    i++;
                    cnt++;
                }

                this->status = AcknowledgeStatus::OK;
                offset       = 0;
            } else {
                this->status = AcknowledgeStatus::IHEX_OK;
                offset       = i;
            }
        } else {
            this->status = status;
        }
    }

    AcknowledgeString(
        uint8_t           sequence,
        const Message&    message,
        AcknowledgeStatus status,
        const char*       string
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message.command);
        std::size_t i = 0;

        while (string[i] != 0) {
            this->data[i] = string[i];
            i++;
        }

        while (i < sizeof(this->data)) {
            this->data[i] = 0;
            i++;
        }

        this->status = status;
    }
}

CORE_PACKED_ALIGNED;
}
