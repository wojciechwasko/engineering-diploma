#include <stdint.h>
#include <boost/crc.hpp>

#include "utils.hpp"

long SeekurJrRC::Utils::gTODDiffToMsec(const struct timeval* t2, const struct timeval* t1)
{
  uint64_t t1_int = 1000*t1->tv_sec + t1->tv_usec/1000;
  uint64_t t2_int = 1000*t2->tv_sec + t2->tv_usec/1000;
  return (long) (t2_int - t1_int);
}


uint32_t SeekurJrRC::Utils::getCrc32(const uint8_t* data, uint32_t length)
{
    boost::crc_32_type result;
    result.process_bytes(data, length);
    return result.checksum();
}

