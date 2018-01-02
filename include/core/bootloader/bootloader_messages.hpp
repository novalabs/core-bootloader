/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/bootloader/bootloader.hpp>

namespace bootloader {
namespace payload {
struct EMPTY {};

struct NAME {
    ModuleName name;
};

struct UID {
    ModuleUID uid;
};

struct UIDAndMaster {
    ModuleUID uid;
    uint8_t   masterID;
};

struct UIDAndCRC {
    ModuleUID uid;
    uint32_t  crc;
};

struct UIDAndID {
    ModuleUID uid;
    uint8_t   id;
};

struct UIDAndName {
    ModuleUID  uid;
    ModuleName name;
};

struct UIDAndAddress {
    ModuleUID uid;
    uint32_t  address;
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
    ModuleUID uid;
};

struct DescribeV1 {
    uint32_t   programFlashSize;
    uint16_t   userFlashSize;
    uint8_t    moduleId;
    ModuleType moduleType;
    ModuleName moduleName;
};

struct DescribeV2 {
    uint32_t   programFlashSize;
    uint32_t   confCRC;
    uint32_t   flashCRC;
    uint16_t   userFlashSize;
    uint8_t    moduleId;
    ModuleType moduleType;
    ModuleName moduleName;
};

struct DescribeV3 {
    uint32_t   programFlashSize;
    uint16_t   userFlashSize;
    uint16_t   tagsFlashSize;
    uint8_t    programValid;
    uint8_t    userValid;
    uint8_t    moduleId;
    ModuleType moduleType;
    ModuleName moduleName;
};
}

namespace messages {
// SLAVE -> MASTER
using Announce = Message_<ShortMessage, MessageType::REQUEST, payload::Announce>;

// MASTER -> SLAVE
using Bootload       = Message_<LongMessage, MessageType::BOOTLOAD, payload::EMPTY>;
using BootloadByName = Message_<LongMessage, MessageType::BOOTLOAD_BY_NAME, payload::NAME>;

using IdentifySlave = Message_<LongMessage, MessageType::IDENTIFY_SLAVE, payload::UID>;
using SelectSlave   = Message_<LongMessage, MessageType::SELECT_SLAVE, payload::UIDAndMaster>;
using DeselectSlave = Message_<LongMessage, MessageType::DESELECT_SLAVE, payload::UID>;

using EraseConfiguration = Message_<LongMessage, MessageType::ERASE_CONFIGURATION, payload::UID>;
using EraseProgram       = Message_<LongMessage, MessageType::ERASE_PROGRAM, payload::UID>;
using WriteProgramCrc    = Message_<LongMessage, MessageType::WRITE_PROGRAM_CRC, payload::UIDAndCRC>;
using DescribeV1 = Message_<LongMessage, MessageType::DESCRIBE_V1, payload::UID>;
using DescribeV2   = Message_<LongMessage, MessageType::DESCRIBE_V2, payload::UID>;
using DescribeV3   = Message_<LongMessage, MessageType::DESCRIBE_V3, payload::UID>;

using TagsRead = Message_<LongMessage, MessageType::TAGS_READ, payload::UIDAndAddress>;

using IHexData = Message_<LongMessage, MessageType::IHEX_READ, payload::IHex>;

using IHexRead = Message_<LongMessage, MessageType::IHEX_WRITE, payload::UIDAndAddress>;

using Reset = Message_<LongMessage, MessageType::RESET, payload::UID>;

// using ReadName = Message_<LongMessage, MessageType::READ_MODULE_NAME, payload::UID>;
using WriteModuleName = Message_<LongMessage, MessageType::WRITE_MODULE_NAME, payload::UIDAndName>;
using WriteModuleID   = Message_<LongMessage, MessageType::WRITE_MODULE_CAN_ID, payload::UIDAndID>;
}


class AcknowledgeUID:
    public AcknowledgeMessage_<LongMessage, payload::UID>
{
public:
    AcknowledgeUID(
        uint8_t           sequence,
        const Message*    message,
        AcknowledgeStatus status,
        ModuleUID         uid
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message->command);
        this->data.uid   = uid;
        this->status     = status;
    }

    AcknowledgeUID(
        uint8_t           sequence,
        const Message&    message,
        AcknowledgeStatus status,
        ModuleUID         uid
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message.command);
        this->data.uid   = uid;
        this->status     = status;
    }
}

CORE_PACKED_ALIGNED;

class AcknowledgeDescribeV1:
    public AcknowledgeMessage_<LongMessage, payload::DescribeV1>
{
public:
    AcknowledgeDescribeV1(
        uint8_t           sequence,
        const Message*    message,
        AcknowledgeStatus status,
        uint8_t           moduleId,
        const char*       module_type,
        const char*       module_name,
        uint32_t          user_flash_size,
        uint32_t          program_flash_size
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message->command);
        this->status     = status;

        if (status == AcknowledgeStatus::OK) {
            this->data.moduleId = moduleId;
            this->data.moduleType.copyFrom(module_type);
            this->data.moduleName.copyFrom(module_name);
            this->data.userFlashSize    = user_flash_size;
            this->data.programFlashSize = program_flash_size;
        } else {
            memset(&this->data, 0, sizeof(this->data));
        }
    }

    AcknowledgeDescribeV1(
        uint8_t           sequence,
        const Message&    message,
        AcknowledgeStatus status,
        uint8_t           moduleId,
        const char*       module_type,
        const char*       module_name,
        uint32_t          user_flash_size,
        uint32_t          program_flash_size
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message.command);
        this->status     = status;

        if (status == AcknowledgeStatus::OK) {
            this->data.moduleId = moduleId;
            this->data.moduleType.copyFrom(module_type);
            this->data.moduleName.copyFrom(module_name);
            this->data.userFlashSize    = user_flash_size;
            this->data.programFlashSize = program_flash_size;
        } else {
            memset(&this->data, 0, sizeof(this->data));
        }
    }
}

CORE_PACKED_ALIGNED;

class AcknowledgeDescribeV2:
    public AcknowledgeMessage_<LongMessage, payload::DescribeV2>
{
public:
    AcknowledgeDescribeV2(
        uint8_t           sequence,
        const Message*    message,
        AcknowledgeStatus status,
        uint8_t           moduleId,
        const char*       module_type,
        const char*       module_name,
        uint32_t          user_flash_size,
        uint32_t          program_flash_size,
        uint32_t          conf_crc,
        uint32_t          flash_crc
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message->command);
        this->status     = status;

        if (status == AcknowledgeStatus::OK) {
            this->data.moduleId = moduleId;
            this->data.moduleType.copyFrom(module_type);
            this->data.moduleName.copyFrom(module_name);
            this->data.userFlashSize    = user_flash_size;
            this->data.programFlashSize = program_flash_size;
            this->data.confCRC          = conf_crc;
            this->data.flashCRC         = flash_crc;
        } else {
            memset(&this->data, 0, sizeof(this->data));
        }
    }

    AcknowledgeDescribeV2(
        uint8_t           sequence,
        const Message&    message,
        AcknowledgeStatus status,
        uint8_t           moduleId,
        const char*       module_type,
        const char*       module_name,
        uint32_t          user_flash_size,
        uint32_t          program_flash_size,
        uint32_t          conf_crc,
        uint32_t          flash_crc
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message.command);
        this->status     = status;

        if (status == AcknowledgeStatus::OK) {
            this->data.moduleId = moduleId;
            this->data.moduleType.copyFrom(module_type);
            this->data.moduleName.copyFrom(module_name);
            this->data.userFlashSize    = user_flash_size;
            this->data.programFlashSize = program_flash_size;
            this->data.confCRC          = conf_crc;
            this->data.flashCRC         = flash_crc;
        } else {
            memset(&this->data, 0, sizeof(this->data));
        }
    }
}

CORE_PACKED_ALIGNED;

class AcknowledgeDescribeV3:
    public AcknowledgeMessage_<LongMessage, payload::DescribeV3>
{
public:
    AcknowledgeDescribeV3(
        uint8_t           sequence,
        const Message*    message,
        AcknowledgeStatus status,
        uint8_t           moduleId,
        const char*       module_type,
        const char*       module_name,
        uint32_t          user_flash_size,
        uint32_t          program_flash_size,
        uint32_t          tags_flash_size,
        bool              program_valid,
		bool              user_valid
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message->command);
        this->status     = status;

        if (status == AcknowledgeStatus::OK) {
            this->data.moduleId = moduleId;
            this->data.moduleType.copyFrom(module_type);
            this->data.moduleName.copyFrom(module_name);
            this->data.userFlashSize    = user_flash_size;
            this->data.programFlashSize = program_flash_size;
            this->data.tagsFlashSize    = tags_flash_size;
            this->data.programValid     = program_valid ? 1 : 0;
            this->data.userValid        = user_valid ? 1 : 0;
        } else {
            memset(&this->data, 0, sizeof(this->data));
        }
    }

    AcknowledgeDescribeV3(
        uint8_t           sequence,
        const Message&    message,
        AcknowledgeStatus status,
        uint8_t           moduleId,
        const char*       module_type,
        const char*       module_name,
        uint32_t          user_flash_size,
        uint32_t          program_flash_size,
        uint32_t          tags_flash_size,
        bool              program_valid,
		bool              user_valid
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message.command);
        this->status     = status;

        if (status == AcknowledgeStatus::OK) {
            this->data.moduleId = moduleId;
            this->data.moduleType.copyFrom(module_type);
            this->data.moduleName.copyFrom(module_name);
            this->data.userFlashSize    = user_flash_size;
            this->data.programFlashSize = program_flash_size;
            this->data.tagsFlashSize    = tags_flash_size;
            this->data.programValid     = program_valid ? 1 : 0;
            this->data.userValid        = user_valid ? 1 : 0;
        } else {
            memset(&this->data, 0, sizeof(this->data));
        }
    }
}

CORE_PACKED_ALIGNED;

class AcknowledgeString:
    public AcknowledgeMessage_<LongMessage, char[44]>
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

class AcknowledgeTags:
    public AcknowledgeMessage_<LongMessage, char[16]>
{
public:
	AcknowledgeTags(
        uint8_t           sequence,
        const Message*    message,
        AcknowledgeStatus status,
        const char*       string
    )
    {
        this->sequenceId = sequence + 1;
        this->type       = static_cast<MessageType>(message->command);
        std::size_t i = 0;

        while (i < sizeof(this->data)) {
            this->data[i] = string[i];
            i++;
        }

        this->status = status;
    }
}

CORE_PACKED_ALIGNED;
}
