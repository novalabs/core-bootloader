/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <nil.h>
#include <hal.h>

#include <cstring>

#include <core/bootloader/bootloader.hpp>
#include <core/bootloader/bootloader_messages.hpp>
#include <core/bootloader/blinker.hpp>
#include <core/bootloader/hw/hw_utils.hpp>
#include "kk_ihex/kk_ihex.h"
#include "kk_ihex/kk_ihex_read.h"
#include "kk_ihex/kk_ihex_write.h"
#include <rtcan.h>
#include <core/stm32_flash/ConfigurationStorage.hpp>
#include <core/stm32_flash/ProgramStorage.hpp>
#include <core/stm32_crc/CRC.hpp>

static const uint8_t led_waiting[] = {
    LED_ON(75), LED_OFF(300), LED_LOOP()
};
static const uint8_t led_initialized[] = {
    LED_ON(75), LED_OFF(100), LED_ON(75), LED_OFF(300), LED_LOOP()
};
static const uint8_t led_booting[] = {
    LED_ON(75), LED_OFF(100), LED_ON(75), LED_OFF(100), LED_ON(75), LED_OFF(300), LED_LOOP()
};
static const uint8_t led_loading[] = {
    LED_ON(200), LED_OFF(100), LED_ON(200), LED_OFF(500), LED_LOOP()
};
static const uint8_t led_muted[] = {
    LED_ON(200), LED_OFF(1000), LED_LOOP()
};
static const uint8_t led_identify[] = {
    LED_ON(100), LED_OFF(100), LED_LOOP()
};
static const uint8_t led_selected[] = {
    LED_ON(100), LED_OFF(100), LED_LOOP()
};

static thread_reference_t trp = NULL;
static const auto         RESUME_BOOTLOADER     = (msg_t)0xCACCAB0B;
static const char         DEFAULT_MODULE_NAME[] = CORE_MODULE_NAME;

static core::stm32_flash::FlashSegment   _programFlash  = core::stm32_flash::FlashSegment(core::stm32_flash::PROGRAM_FLASH_FROM, core::stm32_flash::PROGRAM_FLASH_TO);
static core::stm32_flash::ProgramStorage programStorage = core::stm32_flash::ProgramStorage(_programFlash);

static core::stm32_flash::FlashSegment         _configurationBank1(core::stm32_flash::CONFIGURATION1_FLASH_FROM, core::stm32_flash::CONFIGURATION1_FLASH_TO);
static core::stm32_flash::FlashSegment         _configurationBank2(core::stm32_flash::CONFIGURATION2_FLASH_FROM, core::stm32_flash::CONFIGURATION2_FLASH_TO);
static core::stm32_flash::Storage              _userStorage(_configurationBank1, _configurationBank2);
static core::stm32_flash::ConfigurationStorage configurationStorage(_userStorage);

static bool flashWriteSuccess = false; // Will be = true in eraseProgram (as we do not know if the program flash has already been cleared...

static char   ihexBuffer[256];
static size_t ihexBufferReadOffset = 0;
static char*  ihexBufferWPtr       = nullptr;

template<typename T, T POLY, T DEFAULT = POLY>
struct LFSR {
public:
        using Type = T;
        static constexpr Type POLYNOMIAL() { return POLY; };

  LFSR() : _x(DEFAULT) {}
  LFSR(Type x) : _x(x) {};

  inline Type operator()() {
      return update();
  };

  inline Type operator()(Type x) {
      _x = x;

    return update();
  };

  inline Type current() {
      return _x;
  }

  inline Type next() {
      return update();
  }

private:
  Type _x;
  inline Type update() {
      if(DEFAULT != 0) {
          if(_x == 0) {
              _x = DEFAULT;
          }
      }

      if(_x & 1) {
              _x >>= 1;
          } else {
              _x >>= 1;
              _x ^= POLY;
          }

          return _x;

  }
};

LFSR<uint16_t, 0x82EEu> rng(hw::getUID()[0] << 8 | hw::getUID()[2]);

ihex_bool_t
ihex_data_read(
    struct ihex_state* ihex,
    ihex_record_type_t type,
    ihex_bool_t        error
)
{
    if (error) {
        osalSysHalt("IHex Checksum error");
        flashWriteSuccess = false;
    }

    if (type == IHEX_DATA_RECORD) {
        unsigned long address = (unsigned long)IHEX_LINEAR_ADDRESS(ihex);

        if ((address & 0x00000001) != 0) {
            osalSysHalt("IHex Address misalignment");
            flashWriteSuccess = false;
        }

        if ((ihex->length & 0x01) != 0) {
            osalSysHalt("IHex Length not multiple of 16 bit");
            flashWriteSuccess = false;
        }

        uint16_t data;
        uint8_t* x = (uint8_t*)((void*)(&data));

        if (flashWriteSuccess) {
            // We can write, as everything went well up to now
            for (int i = 0; i < ihex->length; i += 2) {
                // Write every word
                x[0] = ihex->data[i];
                x[1] = ihex->data[i + 1];

                if (programStorage.isAddressValid(address)) {
                    // We want to write into flash
                    if (!programStorage.isReady()) {
                        programStorage.beginWrite();
                    }

                    flashWriteSuccess &= programStorage.write16(address, data);
                } else if (configurationStorage.isUserAddressValid(address)) {
                    // We want to write into user storage
                    if (!configurationStorage.isReady()) {
                        configurationStorage.beginWrite();
                    }

                    flashWriteSuccess &= configurationStorage.writeUserData16(address, data);
                } else {
                    // We want to write in a not allowed location
                    flashWriteSuccess = false;
                }

                address += 2;
            }
        }

        return true;
    } else if (type == IHEX_END_OF_FILE_RECORD) {
        return true;
    } else if (type == IHEX_EXTENDED_LINEAR_ADDRESS_RECORD) {
        return true;
    } else if (type == IHEX_EXTENDED_SEGMENT_ADDRESS_RECORD) {
        return true;
    }

    return false;
} // ihex_data_read

void
ihex_flush_buffer(
    struct ihex_state* ihex,
    char*              buffer,
    char*              eptr
)
{
    *eptr = '\0';

    while (buffer != eptr) {
        // just copy the buffer into a local copy
        *ihexBufferWPtr++ = *buffer++;
    }
}

RTCANConfig rtcan_config = {
    1000000, 100, 60
};

uint8_t data[48];

namespace bootloader {
class SlaveProtocol;

struct IProtocolTransport {
public:
    virtual bool
    initializeTransport(
        SlaveProtocol*
    ) = 0;

    virtual bool
    isInitialized() = 0;

    virtual bool
    transmit(
        const Message* m
    ) = 0;

    virtual bool
    receive(
        Message* m
    ) = 0;
};

class SlaveProtocol
{
public:
    SlaveProtocol(
        IProtocolTransport& transport
    ) :
        _selected(false),
        _sequence(0),
        _muted(false),
        _loading(false),
        _transport(transport),
        _ihex()
    {}

public:
    bool
    initialize()
    {
        bool success = true;

        success &= _transport.initializeTransport(this);

        blinkerSetActive(true);
        updateLed();

        return success;
    }

    void
    start()
    {
        if (_transport.isInitialized()) {
            _loading = true;
        }

        updateLed();
    }

    void
    wait()
    {
        _loading = false;

        updateLed();
    }

public:
    void
    processBootloadMessage(
        const void* message = nullptr
    )
    {
        AcknowledgeStatus status = AcknowledgeStatus::DISCARD;

        Message rxMessage;

        const Message* inMessage;

        if (message == nullptr) {
            inMessage = &rxMessage;

            if (!_transport.receive(&rxMessage)) {
                return;
            }
        } else {
            inMessage = reinterpret_cast<const Message*>(message);
        }

        switch (inMessage->command) {
          case MessageType::BOOTLOAD:
              status = bootloadMessage(inMessage);
              break;
          case MessageType::BOOTLOAD_BY_NAME:
              status = bootloadByNameMessage(inMessage);
              break;
          default:
              break;
        } // switch

        (void)status;
    } // processBootloadMessage

    void
    processMessage(
        const void* message = nullptr
    )
    {
        AcknowledgeStatus status = AcknowledgeStatus::DISCARD;

        Message rxMessage;

        const Message* inMessage;

        if (message == nullptr) {
            inMessage = &rxMessage;

            if (!_transport.receive(&rxMessage)) {
                return;
            }
        } else {
            inMessage = reinterpret_cast<const Message*>(message);
        }

#ifdef LOOPBACK
        _sequence = m->sequenceId;
        AcknowledgeMessage ack(_sequence, m, AcknowledgeStatus::OK);
        _transport.transmit(ack.asMessage());
#else
        switch (inMessage->command) {
          // implemented as a switch to keep it simple...
          case MessageType::REQUEST:

              while (1) {}

              break;
          case MessageType::ACK:
              status = AcknowledgeStatus::DISCARD;
              break;
          case MessageType::IHEX_WRITE:
              status = iHexWriteMessage(inMessage);
              break;
          case MessageType::IHEX_READ:
              status = iHexReadMessage(inMessage);
              break;
          case MessageType::IDENTIFY_SLAVE:
              status = identifyMessage(inMessage);
              break;
          case MessageType::SELECT_SLAVE:
              status = selectMessage(inMessage);
              break;
          case MessageType::DESELECT_SLAVE:
              status = deselectMessage(inMessage);
              break;
          case MessageType::ERASE_CONFIGURATION:
              status = eraseConfigurationMessage(inMessage);
              break;
          case MessageType::ERASE_USER_CONFIGURATION:
              status = eraseUserConfigurationMessage(inMessage);
              break;
          case MessageType::ERASE_PROGRAM:
              status = eraseProgramMessage(inMessage);
              break;
          case MessageType::WRITE_PROGRAM_CRC:
              status = writeProgramCRCMessage(inMessage);
              break;
          case MessageType::WRITE_MODULE_NAME:
              status = writeModuleNameMessage(inMessage);
              break;
          case MessageType::WRITE_MODULE_ID:
              status = writeModuleIDMessaqe(inMessage);
              break;
          case MessageType::BOOTLOAD:
              status = AcknowledgeStatus::DISCARD;
              break;
          case MessageType::RESET:
              status = resetMessage(inMessage);
              break;
          default:
              status = AcknowledgeStatus::DISCARD;
        } // switch

        if ((status != AcknowledgeStatus::DISCARD) && (status != AcknowledgeStatus::DO_NOT_ACK)) {
            switch (inMessage->command) {
              // implemented as a switch to keep it simple...
              case MessageType::IHEX_READ:
              {
                  AcknowledgeString txMessage = AcknowledgeString(_sequence, inMessage, status, ihexBuffer, ihexBufferReadOffset);
                  _transport.transmit(txMessage.asMessage());
              }
              break;
              case MessageType::IHEX_WRITE:
              case MessageType::IDENTIFY_SLAVE:
              case MessageType::SELECT_SLAVE:
              case MessageType::DESELECT_SLAVE:
              case MessageType::ERASE_CONFIGURATION:
              case MessageType::ERASE_USER_CONFIGURATION:
              case MessageType::ERASE_PROGRAM:
              case MessageType::WRITE_PROGRAM_CRC:
              case MessageType::WRITE_MODULE_NAME:
              case MessageType::WRITE_MODULE_ID:
              case MessageType::RESET:
              default:
              {
                  AcknowledgeUID txMessage = AcknowledgeUID(_sequence, inMessage, status);
                  _transport.transmit(txMessage.asMessage());
              }
            } // switch
        } else if (status == AcknowledgeStatus::DISCARD) {} else {
            // The message was not for us...
        }
#endif // ifdef LOOPBACK
    } // processMessage

public:
    void
    announce()
    {
        if (!_selected && !_muted) {
            messages::Announce m;
            m.data.uid = hw::getUID();
            m.data.userFlashSize    = configurationStorage.userDataSize();
            m.data.programFlashSize = programStorage.size() / 32;
            m.data.moduleId = configurationStorage.getModuleConfiguration()->moduleID;
            m.data.moduleType.copyFrom(CORE_MODULE_NAME);
            m.data.moduleName.copyFrom(configurationStorage.getModuleConfiguration()->name);
            _transport.transmit(&m);
        }
    }

public:
    AcknowledgeStatus
    identifyMessage(
        const Message* message
    )
    {
        const messages::IdentifySlave* m = reinterpret_cast<const messages::IdentifySlave*>(message);

        if (m->data.uid == hw::getUID()) {
            return identify(true);
        } else {
            return identify(false);
        }
    }

    AcknowledgeStatus
    selectMessage(
        const Message* message
    )
    {
        const messages::SelectSlave* m = reinterpret_cast<const messages::SelectSlave*>(message);

        if (m->data.uid == hw::getUID()) {
            _sequence = m->sequenceId; // The sequence number is re-aligned
            return select();
        } else {
            // The master selected another slave, we must deselct and mute ourselves
            deselect();
            mute(true);
            return AcknowledgeStatus::DISCARD;
        }
    } // selectMessage

    AcknowledgeStatus
    deselectMessage(
        const Message* message
    )
    {
        const messages::DeselectSlave* m = reinterpret_cast<const messages::DeselectSlave*>(message);

        if (m->data.uid == hw::getUID()) {
            if (_selected) {
                if (m->sequenceId != (uint8_t)(_sequence + 2)) {
                    return AcknowledgeStatus::WRONG_SEQUENCE;
                } else {
                    _sequence = m->sequenceId;
                    return deselect();
                }
            } else {
                return AcknowledgeStatus::DISCARD;
            }
        } else {
            if (_selected) {
                return AcknowledgeStatus::WRONG_UID;
            } else {
                // The message was for someone else... We can now start again to advertise
                mute(false);
                return AcknowledgeStatus::DISCARD;
            }
        }
    } // deselectMessage

    AcknowledgeStatus
    eraseConfigurationMessage(
        const Message* message
    )
    {
        const messages::EraseConfiguration* m = reinterpret_cast<const messages::EraseConfiguration*>(message);

        if (m->data.uid == hw::getUID()) {
            if (_selected) {
                if (m->sequenceId != (uint8_t)(_sequence + 2)) {
                    return AcknowledgeStatus::WRONG_SEQUENCE;
                } else {
                    _sequence = m->sequenceId;
                    return eraseConfiguration();
                }
            } else {
                return AcknowledgeStatus::NOT_SELECTED;
            }
        } else {
            if (_selected) {
                return AcknowledgeStatus::WRONG_UID;
            } else {
                return AcknowledgeStatus::DISCARD;
            }
        }
    } // eraseConfigurationMessage

    AcknowledgeStatus
    eraseUserConfigurationMessage(
        const Message* message
    )
    {
        const messages::EraseConfiguration* m = reinterpret_cast<const messages::EraseConfiguration*>(message);

        if (m->data.uid == hw::getUID()) {
            if (_selected) {
                if (m->sequenceId != (uint8_t)(_sequence + 2)) {
                    return AcknowledgeStatus::WRONG_SEQUENCE;
                } else {
                    _sequence = m->sequenceId;
                    return eraseUserConfiguration();
                }
            } else {
                return AcknowledgeStatus::NOT_SELECTED;
            }
        } else {
            if (_selected) {
                return AcknowledgeStatus::WRONG_UID;
            } else {
                return AcknowledgeStatus::DISCARD;
            }
        }
    } // eraseUserConfigurationMessage

    AcknowledgeStatus
    eraseProgramMessage(
        const Message* message
    )
    {
        const messages::EraseProgram* m = reinterpret_cast<const messages::EraseProgram*>(message);

        if (m->data.uid == hw::getUID()) {
            if (_selected) {
                if (m->sequenceId != (uint8_t)(_sequence + 2)) {
                    return AcknowledgeStatus::WRONG_SEQUENCE;
                } else {
                    _sequence = m->sequenceId;
                    return eraseProgram();
                }
            } else {
                return AcknowledgeStatus::NOT_SELECTED;
            }
        } else {
            if (_selected) {
                return AcknowledgeStatus::WRONG_UID;
            } else {
                return AcknowledgeStatus::DISCARD;
            }
        }
    } // eraseProgramMessage

    AcknowledgeStatus
    writeProgramCRCMessage(
        const Message* message
    )
    {
        const messages::WriteProgramCrc* m = reinterpret_cast<const messages::WriteProgramCrc*>(message);

        if (m->data.uid == hw::getUID()) {
            if (_selected) {
                if (m->sequenceId != (uint8_t)(_sequence + 2)) {
                    return AcknowledgeStatus::WRONG_SEQUENCE;
                } else {
                    _sequence = m->sequenceId;
                    return writeProgramCRC(m->data.crc);
                }
            } else {
                return AcknowledgeStatus::NOT_SELECTED;
            }
        } else {
            if (_selected) {
                return AcknowledgeStatus::WRONG_UID;
            } else {
                return AcknowledgeStatus::DISCARD;
            }
        }
    } // writeProgramCRCMessage

    AcknowledgeStatus
    writeModuleNameMessage(
        const Message* message
    )
    {
        const messages::WriteModuleName* m = reinterpret_cast<const messages::WriteModuleName*>(message);

        if (m->data.uid == hw::getUID()) {
            if (_selected) {
                if (m->sequenceId != (uint8_t)(_sequence + 2)) {
                    return AcknowledgeStatus::WRONG_SEQUENCE;
                } else {
                    _sequence = m->sequenceId;
                    return writeModuleName(m->data.name);
                }
            } else {
                return AcknowledgeStatus::NOT_SELECTED;
            }
        } else {
            if (_selected) {
                return AcknowledgeStatus::WRONG_UID;
            } else {
                return AcknowledgeStatus::DISCARD;
            }
        }
    } // writeModuleNameMessage

    AcknowledgeStatus
    writeModuleIDMessaqe(
        const Message* message
    )
    {
        const messages::WriteModuleID* m = reinterpret_cast<const messages::WriteModuleID*>(message);

        if (m->data.uid == hw::getUID()) {
            if (_selected) {
                if (m->sequenceId != (uint8_t)(_sequence + 2)) {
                    return AcknowledgeStatus::WRONG_SEQUENCE;
                } else {
                    _sequence = m->sequenceId;
                    return writeModuleID(m->data.id);
                }
            } else {
                return AcknowledgeStatus::NOT_SELECTED;
            }
        } else {
            if (_selected) {
                return AcknowledgeStatus::WRONG_UID;
            } else {
                return AcknowledgeStatus::DISCARD;
            }
        }
    } // writeProgramCRCMessage

    AcknowledgeStatus
    iHexWriteMessage(
        const Message* message
    )
    {
        const messages::IHexData* m = reinterpret_cast<const messages::IHexData*>(message);

        if (_selected) {
            if (m->sequenceId != (uint8_t)(_sequence + 2)) {
                return AcknowledgeStatus::WRONG_SEQUENCE;
            } else {
                _sequence = m->sequenceId;
                return ihexWrite(m->data.type, m->data.string);
            }
        } else {
            return AcknowledgeStatus::DISCARD;
        }
    }

    AcknowledgeStatus
    iHexReadMessage(
        const Message* message
    )
    {
        const messages::IHexRead* m = reinterpret_cast<const messages::IHexRead*>(message);

        if (m->data.uid == hw::getUID()) {
            if (_selected) {
                if (m->sequenceId != (uint8_t)(_sequence + 2)) {
                    return AcknowledgeStatus::WRONG_SEQUENCE;
                } else {
                    _sequence = m->sequenceId;

                    if (m->data.address == 0xFFFFFFFF) {
                        // We want to continue to read the buffer
                        return AcknowledgeStatus::OK;
                    } else {
                        ihexBufferReadOffset = 0; // reset the read buffer offset
                        return ihexRead(m->data.address, ihexBuffer);
                    }
                }
            } else {
                return AcknowledgeStatus::NOT_SELECTED;
            }
        } else {
            if (_selected) {
                return AcknowledgeStatus::WRONG_UID;
            } else {
                return AcknowledgeStatus::DISCARD;
            }
        }
    } // iHexReadMessage

    AcknowledgeStatus
    bootloadMessage(
        const Message* message
    )
    {
//			const messages::Bootload* m = reinterpret_cast<const messages::Bootload*>(message);
        return bootload();
    }

    AcknowledgeStatus
    bootloadByNameMessage(
        const Message* message
    )
    {
        const messages::BootloadByName* m = reinterpret_cast<const messages::BootloadByName*>(message);

        if (m->data.name == configurationStorage.getModuleConfiguration()->name) {
            return bootload();
        }

        return AcknowledgeStatus::DISCARD;
    }

    AcknowledgeStatus
    resetMessage(
        const Message* message
    )
    {
        const messages::Reset* m = reinterpret_cast<const messages::Reset*>(message);

        if (m->data.uid == hw::getUID()) {
            if (_selected) {
                if (m->sequenceId != (uint8_t)(_sequence + 2)) {
                    return AcknowledgeStatus::WRONG_SEQUENCE;
                } else {
                    _sequence = m->sequenceId;
                    return reset();
                }
            } else {
                return AcknowledgeStatus::NOT_SELECTED;
            }
        } else {
            if (_selected) {
                return AcknowledgeStatus::WRONG_UID;
            } else {
                return AcknowledgeStatus::DISCARD;
            }
        }
    } // resetMessage

    AcknowledgeStatus
    identify(
        bool me
    )
    {
        if (me) {
            if (!_selected) {
                blinkerSetPattern(led_identify);
                return AcknowledgeStatus::OK;
            }
        }

        updateLed();

        return AcknowledgeStatus::DISCARD;
    }

    AcknowledgeStatus
    select()
    {
        _selected = true;
        _muted    = false;

        updateLed();

        return AcknowledgeStatus::OK;
    }

    AcknowledgeStatus
    deselect()
    {
        _selected = false;

        updateLed();

        return AcknowledgeStatus::OK;
    }

    AcknowledgeStatus
    eraseConfiguration()
    {
        if (configurationStorage.erase()) {
            return AcknowledgeStatus::OK;
        } else {
            return AcknowledgeStatus::ERROR;
        }
    }

    AcknowledgeStatus
    eraseUserConfiguration()
    {
        if (configurationStorage.unlock() && configurationStorage.eraseUserConfiguration()) {
            flashWriteSuccess = true;
            return AcknowledgeStatus::OK;
        } else {
            return AcknowledgeStatus::ERROR;
        }
    }

    AcknowledgeStatus
    eraseProgram()
    {
        if (programStorage.unlock() && programStorage.erase()) {
            flashWriteSuccess = true;
            return AcknowledgeStatus::OK;
        } else {
            return AcknowledgeStatus::ERROR;
        }

        return AcknowledgeStatus::OK;
    }

    AcknowledgeStatus
    writeProgramCRC(
        uint32_t crc
    )
    {
        if (configurationStorage.writeProgramCRC(crc)) {
            return AcknowledgeStatus::OK;
        } else {
            return AcknowledgeStatus::ERROR;
        }

        return AcknowledgeStatus::OK;
    }

    AcknowledgeStatus
    writeModuleName(
        ModuleName name
    )
    {
        if (configurationStorage.writeModuleName(reinterpret_cast<const char*>(name.data()))) {
            return AcknowledgeStatus::OK;
        } else {
            return AcknowledgeStatus::ERROR;
        }

        return AcknowledgeStatus::OK;
    }

    AcknowledgeStatus
    writeModuleID(
        uint8_t id
    )
    {
        if (configurationStorage.writeModuleID(id)) {
            return AcknowledgeStatus::OK;
        } else {
            return AcknowledgeStatus::ERROR;
        }

        return AcknowledgeStatus::OK;
    }

    AcknowledgeStatus
    ihexWrite(
        payload::IHex::Type type,
        const char          string[44]
    )
    {
        const char* data = string;

        switch (type) {
          case payload::IHex::Type::BEGIN:
              blinkerSetActive(false);
              flashWriteSuccess = true; // Reset the success flag
              ihex_begin_read(&_ihex);
              break;
          case payload::IHex::Type::DATA:
              blinkerForce(true);
              ihex_read_bytes(&_ihex, data, std::strlen(data));
              blinkerForce(false);
              break;
          case payload::IHex::Type::END:
              ihex_end_read(&_ihex);

              if (programStorage.isReady()) {
                  flashWriteSuccess &= programStorage.endWrite();
              }

              if (configurationStorage.isReady()) {
                  flashWriteSuccess &= configurationStorage.endWrite();
              }

              blinkerSetActive(true);

              if (flashWriteSuccess) {
                  return AcknowledgeStatus::OK;
              } else {
                  return AcknowledgeStatus::ERROR;
              }

              flashWriteSuccess = false; // Prohibit any attempt to write again

              break;
          default:
              return AcknowledgeStatus::BROKEN;
        } // switch

        if (flashWriteSuccess) {
            return AcknowledgeStatus::OK;
        } else {
            return AcknowledgeStatus::ERROR;
        }
    } // ihexWrite

    AcknowledgeStatus
    ihexRead(
        uint32_t address,
        char*    buffer
    )
    {
        ihexBufferWPtr = buffer;

        ihex_init(&_ihex);
        ihex_write_at_address(&_ihex, address);

        if (programStorage.isAddressValid(address)) {
            ihex_write_bytes(&_ihex, reinterpret_cast<void*>(address), 16);
            ihex_set_output_line_length(&_ihex, 16);
        } else if (configurationStorage.isUserAddressValid(address)) {
            ihex_write_bytes(&_ihex, reinterpret_cast<void*>(((uint32_t)configurationStorage.getUserConfiguration()) + address), 16);
            ihex_set_output_line_length(&_ihex, 16);
        } else {
            return AcknowledgeStatus::ERROR;
        }

        ihex_end_write(&_ihex);

        return AcknowledgeStatus::OK;
    } // ihexRead

    AcknowledgeStatus
    bootload()
    {
        hw::setNVR(hw::WatchdogReason::USER_REQUEST);
        hw::watchdogEnable(hw::watchdogPeriod::PERIOD_800_MS);

        while (1) {}
    }

    AcknowledgeStatus
    reset()
    {
        hw::reset();

        return AcknowledgeStatus::OK;
    }

private:
    inline bool
    isMuted()
    {
        return _muted;
    }

    void
    updateLed()
    {
        if (_transport.isInitialized()) {
            if (_loading) {
                if (_selected) {
                    blinkerSetPattern(led_selected);
                } else {
                    if (_muted) {
                        blinkerSetPattern(led_muted);
                    } else {
                        blinkerSetPattern(led_loading);
                    }
                }
            } else {
                blinkerSetPattern(led_initialized);
            }
        } else {
            blinkerSetPattern(led_waiting);
        }
    } // updateLed

    void
    mute(
        bool muted
    )
    {
        if (!_selected) {
            _muted = muted;
        } else {
            _muted = false;
        }

        updateLed();
    }

private:
    bool    _selected;
    uint8_t _sequence;
    bool    _muted;
    bool    _loading;
    IProtocolTransport& _transport;
    ihex_state          _ihex;
}

;

class CANTransport:
    public IProtocolTransport
{
public:
    CANTransport() :
        _readBuffer(nullptr),
        _filterId(0x0000),
        _state(State::INITIALIZING)

    {}

    ~CANTransport()
    {
        rtcanStop(&RTCAND1);
    }

    bool
    isInitialized()
    {
        return _state == State::INITIALIZED;
    }

    bool
    initializeTransport(
        SlaveProtocol* protocol
    )
    {
        rtcanInit();
        rtcanStart(&RTCAND1, &rtcan_config);

        return true;
    } // initializeTransport

    bool
    transmit(
        const Message* m
    )
    {
        if (_state == State::INITIALIZED) {
            m->copyTo(_bufferTx);

            rtcanTransmit(&RTCAND1, &_messageTx, MS2ST(rng())); // The timeout is random

            return true;
        } else {
            return false;
        }
    }

    bool
    receive(
        Message* m
    )
    {
        if (_state == State::INITIALIZED) {
            if (_readBuffer == nullptr) {
                return false;
            }

            m->copyFrom(_readBuffer);

            _readBuffer = nullptr; // swap() will set it!

            return true;
        } else {
            return false;
        }
    }

    bool
    waitForMaster()
    {
        if (_state != State::INITIALIZING) {
            return false;
        }

        rtcan_msg_t* rtcan_msg_p;

        rtcan_msg_p           = &_messageRx;
        rtcan_msg_p->id       = 0xFC00; // Bootloader Master
        rtcan_msg_p->callback = reinterpret_cast<rtcan_msgcallback_t>(CANTransport::recv_cb);
        rtcan_msg_p->params   = this;
        rtcan_msg_p->size     = 8;
        rtcan_msg_p->data     = reinterpret_cast<uint8_t*>(&_bufferRx0);
        rtcan_msg_p->status   = RTCAN_MSG_READY;

        rtcanReceiveMask(&RTCAND1, &_messageRx, 0xFF00);

        return true;
    } // waitForMaster

    bool
    setFilter()
    {
        if (_state != State::INITIALIZED) {
            return false;
        }

        rtcan_msg_t* rtcan_msg_p;

        uint8_t moduleID = configurationStorage.getModuleConfiguration()->moduleID;

        while(moduleID == 0xFF) {
            moduleID = rng(); // Get a random ID
        }

        rtcan_msg_p           = &_messageTx;
        rtcan_msg_p->id       = 0xFD00 | moduleID;
        rtcan_msg_p->callback = nullptr;
        rtcan_msg_p->params   = this;
        rtcan_msg_p->size     = MESSAGE_LENGTH;
        rtcan_msg_p->data     = reinterpret_cast<uint8_t*>(&_bufferTx);
        rtcan_msg_p->status   = RTCAN_MSG_READY;

        rtcan_msg_p           = &_messageRx;
        rtcan_msg_p->id       = 0xFD00 | _filterId;
        rtcan_msg_p->callback = reinterpret_cast<rtcan_msgcallback_t>(CANTransport::recv_cb);
        rtcan_msg_p->params   = this;
        rtcan_msg_p->size     = MESSAGE_LENGTH;
        rtcan_msg_p->data     = reinterpret_cast<uint8_t*>(&_bufferRx0);
        rtcan_msg_p->status   = RTCAN_MSG_READY;

        rtcanReceive(&RTCAND1, &_messageRx);

        return true;
    } // setFilter

private:
    static void
    recv_cb(
        rtcan_msg_t& rtcan_msg
    )
    {
        CANTransport* _this = reinterpret_cast<CANTransport*>(rtcan_msg.params);

        if (_this->_state == State::INITIALIZING) {
            if (rtcan_msg.status == RTCAN_MSG_BUSY) {
                _this->swap();
                _this->_filterId = rtcan_msg.id & 0x00FF;
                _this->_state    = State::INITIALIZED;
                osalThreadResumeI(&trp, RESUME_BOOTLOADER); // resume the bootloader thread with message
            }

            rtcan_msg.status = RTCAN_MSG_READY; // welcome a new message
        } else if (_this->_state == State::INITIALIZED) {
            if (rtcan_msg.id != (0xFD00 | _this->_filterId)) {
                rtcan_msg.status = RTCAN_MSG_READY; // welcome a new message
                // It was not an interesting message...
                return;
            }

            if (rtcan_msg.status == RTCAN_MSG_BUSY) {
                _this->swap();
                osalThreadResumeI(&trp, RESUME_BOOTLOADER); // resume the bootloader thread with message
            }

            rtcan_msg.status = RTCAN_MSG_READY; // welcome a new message
        }
    } // recv_cb

private:
    uint8_t _bufferTx[MESSAGE_LENGTH];

    uint8_t _bufferRx0[MESSAGE_LENGTH];
    uint8_t _bufferRx1[MESSAGE_LENGTH];

    rtcan_msg_t _messageTx;
    rtcan_msg_t _messageRx;

    uint8_t* _readBuffer;

    rtcan_id_t _filterId;

    enum class State {
        INITIALIZING,
        INITIALIZED
    };

    State _state;

private:
    inline void
    swap()
    {
//			osalSysLockFromISR(); // called from and ISR, it is already locked!
        uint8_t* buffer0 = reinterpret_cast<uint8_t*>(&_bufferRx0);
        uint8_t* buffer1 = reinterpret_cast<uint8_t*>(&_bufferRx1);

        if (_messageRx.data == buffer0) {
            _messageRx.data = buffer1;
            _readBuffer     = buffer0;
        } else {
            _messageRx.data = buffer0;
            _readBuffer     = buffer1;
        }

//			osalSysUnlockFromISR();
    }
};
}

#define FORCE_LOADER    false
#define OVERRIDE_LOADER false

void
boot()
{
    blinkerSetActive(true);
    blinkerSetPattern(led_booting);

    core::stm32_crc::CRC::init();
    core::stm32_crc::CRC::setPolynomialSize(core::stm32_crc::CRC::PolynomialSize::POLY_32);
    core::stm32_crc::CRC::CRCBlock((uint32_t*)programStorage.from(), programStorage.size() / sizeof(uint32_t));

    hw::setNVR(hw::WatchdogReason::NO_APPLICATION);

    volatile uint32_t imageCRC = configurationStorage.getModuleConfiguration()->imageCRC;
    volatile uint32_t flashCRC =  core::stm32_crc::CRC::getCRC();

    if (imageCRC != flashCRC) {
        // The image is broken, do not even try to run it!!!
        hw::watchdogEnable(hw::watchdogPeriod::PERIOD_1600_MS);

        while (1) {
            osalThreadSleepMilliseconds(2000);
        }
    }

    // ... try to boot the App!
    if (!OVERRIDE_LOADER) {
        hw::watchdogEnable(hw::watchdogPeriod::PERIOD_6400_MS); // give the App some time to start...
    }

    rtcanStop(&RTCAND1);

    hw::jumptoapp(programStorage.from());
} // boot

THD_WORKING_AREA(bootloaderThreadWorkingArea, 4096);
THD_FUNCTION(bootloaderThread, arg) {
    bootloader::CANTransport  transport;
    bootloader::SlaveProtocol proto(transport);

    // Try to make it more random...
    uint8_t cnt;
    cnt = hw::getUID()[0] ^ hw::getUID()[5];
    while(cnt--) {
        rng();
    }

    cnt = hw::getUID()[2] ^ hw::getUID()[7];
    while(cnt--) {
        rng(cnt);
    }

    proto.initialize();

    if (configurationStorage.getModuleConfiguration()->name[0] == 0xFF) {
        // Name is empty, let's fill it with the default
        configurationStorage.writeModuleName(DEFAULT_MODULE_NAME);
    }

#if OVERRIDE_LOADER
    boot();
#else
    hw::watchdogReload();
#endif

    // Depending on how we got here, we must do something different:

    bool tryToBoot = true;
    bool bootload  = false;

#if FORCE_LOADER
    bootload  = true;
    tryToBoot = false;
#else
    if (hw::getResetSource() == hw::ResetSource::WATCHDOG) {
        // The watchdog has resetted the uC, why?
        if (hw::getNVR() == hw::WatchdogReason::BOOT_APPLICATION) {
            // We were explicitely requested to boot the application. Do not even try to init the bootloader protocol
            boot();
        } else if (hw::getNVR() == hw::WatchdogReason::NO_APPLICATION) {
            // We tried to boot, but without success. Do not try again, the result will be the same.
            // Initialize the bootloader and see what happens.
            tryToBoot = false;
            bootload  = false;
        } else if (hw::getNVR() == hw::WatchdogReason::USER_REQUEST) {
            // We were explicitely requested to start the bootloader. So we will not boot the app.
            tryToBoot = false;
            bootload  = true;
        }
    }

    hw::setNVR(hw::WatchdogReason::TRANSPORT_FAIL); // If anything goes wrong, die.
    hw::watchdogEnable(hw::watchdogPeriod::PERIOD_6400_MS); // Take it easy!
#endif // if FORCE_LOADER

    bool waitForMaster = true;

    while (waitForMaster) {
        // Wait for a bootloader master to advertise it's existence
        transport.waitForMaster();

        hw::watchdogReload();

        osalSysLock();
        msg_t msg = osalThreadSuspendTimeoutS(&trp, MS2ST(2000)); // In the meanwhile, sleep.
        osalSysUnlock();

        if (!transport.isInitialized()) {
            // We were waken up, but no master was there...
            if (msg == MSG_TIMEOUT) {
                // ... because it did not responded in time
                if (tryToBoot) {
                    // if we were supposed to boot
                    boot();
                }
            } else {
                // ... shit!
            }
        } else {
            // A master is there. Hail to the master! Stop waiting...
            transport.setFilter();
            waitForMaster = false;
        }
    }

    //bootload = true;

    if (bootload) {
        // We must bootload

        proto.start();

        uint8_t cnt = 0;

#if !FORCE_LOADER
        hw::setNVR(hw::WatchdogReason::BOOT_APPLICATION); // Fallback if the bootloader stops working
        hw::watchdogEnable(hw::watchdogPeriod::PERIOD_6400_MS); // Take it easy!
#endif

        while (1) {
            msg_t msg;

            hw::watchdogReload();

            osalSysLock();

            msg = osalThreadSuspendTimeoutS(&trp, MS2ST(100));

            if (msg != RESUME_BOOTLOADER) {
                if (cnt & 0x0F) {
                    proto.announce();
                }

                cnt++;
            } else {
                proto.processMessage();
            }

            osalSysUnlock();
        }
    } else {
        // We were not requested to bootload...

        proto.wait();

        bool loopForever = false;

        if (hw::getResetSource() == hw::ResetSource::WATCHDOG) {
            // The watchdog brought us here...
            // There is nothing else to do than waiting forever...
            loopForever = true;
        } else {
            // The board has been reset, if we do not receive a bootload request, try to boot the application!
            hw::setNVR(hw::WatchdogReason::BOOT_APPLICATION);
            hw::watchdogEnable(hw::watchdogPeriod::PERIOD_1600_MS); // Give the protocol some time to catch a bootload message...
        }

//        loopForever = true;

        while (1) {
            msg_t msg;

            if (loopForever) {
                hw::watchdogReload();
            }

            osalSysLock();

            msg = osalThreadSuspendTimeoutS(&trp, MS2ST(100));

            if (msg == RESUME_BOOTLOADER) {
                proto.processBootloadMessage();
            }

            osalSysUnlock();
        }
    }

    while (1) {
        // no return function
    }

    // unreachable
}
