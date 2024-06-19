
#include "cybergear_can_interface_Teensy4.hh"

// #ifdef CONFIG_IDF_TARGET_ESP32S3

#include "FlexCAN_T4.h"
#include "cybergear_driver_utils.hh"

CybergearCanInterfaceTeensy::CybergearCanInterfaceTeensy() : CybergearCanInterface() {}

CybergearCanInterfaceTeensy::~CybergearCanInterfaceTeensy() {}

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
// static RingBuf<CAN_message_t, 100> buffer;

bool CybergearCanInterfaceTeensy::init()
{
  // return ESP32Can.begin(ESP32Can.convertSpeed(1000), tx_pin, rx_pin, 10, 10);
  can1.begin();
  can1.setBaudRate(1000000);
  return true;

}

bool CybergearCanInterfaceTeensy::send_message(
  uint32_t id, const uint8_t * data, uint8_t len, bool ext)
{
  CG_DEBUG_FUNC
  // CanFrame frame = {0}
  CAN_message_t sendmsg;
  sendmsg.id = id;
  sendmsg.flags.extended = (ext) ? 1 : 0;
  sendmsg.len = len;
  memcpy(sendmsg.buf, data, len);
  return can1.write(sendmsg);
}

bool CybergearCanInterfaceTeensy::read_message(unsigned long & id, uint8_t * data, uint8_t & len)
{
  CG_DEBUG_FUNC
  // CanFrame frame = {0};
  CAN_message_t readmsg;
  if (!can1.read(readmsg)) return false;

  // get mseesage from buffer
  if (readmsg.flags.remote) return false;

  id = readmsg.id;
  len = readmsg.len;
  memcpy(data, readmsg.buf, readmsg.len);
  return true;
}


bool CybergearCanInterfaceTeensy::available()
{
  CG_DEBUG_FUNC
   CAN_message_t rmsg;
  // return (buffer.isEmpty() == false);
  if(can1.read(rmsg)){
    return true;
  }else{
    return false;
  }

}



// #endif  // CONFIG_IDF_TARGET_ESP32S3
