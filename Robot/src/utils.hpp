#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <sys/time.h>
#include <stdint.h>
#include <string>

namespace SeekurJrRC {
  namespace Utils {
    /// przelicza różnicę pomiędzy dwoma struct timeval
    /// i zwraca ją w milisekundach (t2 - t1)
    long gTODDiffToMsec(const struct timeval* t2, const struct timeval* t1);
    uint32_t getCrc32(const uint8_t* data, uint32_t length);
    inline bool isSystemBigEndian(void)
    {
      union {
        uint32_t i;
        char c[4];
      } b_int = {0x01020304};

      return (b_int.c[0] == 1) ;
    }

    inline void swapEndianness(uint16_t& x)
    {
      x = (x >> 8) |
          (x << 8);
    }

    inline void swapEndianness(uint32_t& x)
    {
      x = (x >> 24) |
          ((x << 8) & 0x00FF0000) |
          ((x >> 8) & 0x0000FF00) |
          (x << 24);
    }

    inline void swapEndianness(uint64_t& x)
    {
      x = (x >> 56) |
          ((x << 40) & 0x00FF000000000000LL) |
          ((x << 24) & 0x0000FF0000000000LL) |
          ((x << 8)  & 0x000000FF00000000LL) |
          ((x >> 8)  & 0x00000000FF000000LL) |
          ((x >> 24) & 0x0000000000FF0000LL) |
          ((x >> 40) & 0x000000000000FF00LL) |
          (x << 56);
    }
  }
}

#endif

