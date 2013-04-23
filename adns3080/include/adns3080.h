#ifndef ADNS3080_H_
#define ADNS3080_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

struct adns3080_output
{
  uint8_t motion;
  int8_t delta_x;
  int8_t delta_y;
  uint8_t squal;
  uint8_t shutter_upper;
  uint8_t shutter_lower;
  uint8_t max_pixel;
};

int adns3080_init(const char *device);
uint8_t adns3080_spi_write(int fd, uint8_t address, uint8_t value);
struct adns3080_output adns3080_read_motion_burst(int fd);

// buf has to be of size > 900 bytes
int adns3080_read_frame(int fd, uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif
