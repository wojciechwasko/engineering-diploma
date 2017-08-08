#include <iostream>
#include <string.h> // for memcpy
#include <stdint.h>

#include "message.hpp"
#include "utils.hpp"

#include <stdio.h>

using SeekurJrRC::Core::TCPMessage;

TCPMessage::TCPMessage(const uint8_t* const buffer) : x_(rw_x_), y_(rw_y_), z_(rw_z_), steeringModelCode_(rw_steeringModelCode_), startStop_(rw_startStop_)
{
  
  uint8_t message_part[offsetCRC32_];
  memcpy(&message_part[0], buffer, offsetCRC32_);
  
  uint32_t packet_crc32_checksum;
  memcpy(&packet_crc32_checksum, buffer + offsetCRC32_, 4);
  
  if (!SeekurJrRC::Utils::isSystemBigEndian())
    SeekurJrRC::Utils::swapEndianness(packet_crc32_checksum);
  
  // debug
  // printf("%08X : %08X => ", SeekurJrRC::Utils::getCrc32(message_part, messageLength_-4), packet_crc32_checksum);
  
  if (SeekurJrRC::Utils::getCrc32(message_part, messageLength_-4) != packet_crc32_checksum)
    throw "Checksums don't check out.";
  
  // używamy unii, bo możliwe, że konieczne będzie przewrócenie bitów!
  union {
    uint32_t i;
    float f;
  } uint32_t_float_conv;
  
  // x value
  memcpy(&uint32_t_float_conv.i, buffer + offsetX_, 4);
  if (!SeekurJrRC::Utils::isSystemBigEndian())
    SeekurJrRC::Utils::swapEndianness(uint32_t_float_conv.i);
  rw_x_ = uint32_t_float_conv.f;
  
  // y value
  memcpy(&uint32_t_float_conv.i, buffer + offsetY_, 4);
  if (!SeekurJrRC::Utils::isSystemBigEndian())
    SeekurJrRC::Utils::swapEndianness(uint32_t_float_conv.i);
  rw_y_ = uint32_t_float_conv.f;
  
  // z value
  memcpy(&uint32_t_float_conv.i, buffer + offsetZ_, 4);
  if (!SeekurJrRC::Utils::isSystemBigEndian())
    SeekurJrRC::Utils::swapEndianness(uint32_t_float_conv.i);
  rw_z_ = uint32_t_float_conv.f;
  
  // dummy for later use
  uint8_t dummy_uint8;
  
  memcpy(&dummy_uint8, buffer + offsetSteeringModelCode_, 1);
  rw_steeringModelCode_ = dummy_uint8;
  
  memcpy(&dummy_uint8, buffer + offsetStartStop_, 1); // we don't have to worry about endianness, if all bits are 1, drive, otherwise, stop
  rw_startStop_ = (dummy_uint8 == ((uint8_t)-1));
}
