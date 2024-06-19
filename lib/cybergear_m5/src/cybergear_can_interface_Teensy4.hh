#ifndef CYBERGEAR_CAN_INTERFACE_TEENSY4_HH
#define CYBERGEAR_CAN_INTERFACE_TEENSY4_HH

#include "cybergear_can_interface.hh"

// #define M5_ESP32_DEFAULT_RX_PIN 1
// #define M5_ESP32_DEFAULT_TX_PIN 2

class CybergearCanInterfaceTeensy : public CybergearCanInterface
{
public:
  CybergearCanInterfaceTeensy();
  virtual ~CybergearCanInterfaceTeensy();
  bool init();
  virtual bool send_message(uint32_t id, const uint8_t * data, uint8_t len, bool ext);
  virtual bool read_message(unsigned long & id, uint8_t * data, uint8_t & len);
  virtual bool available();
};


#endif  // CYBERGEAR_CAN_INTERFACE_TWAI_HH
