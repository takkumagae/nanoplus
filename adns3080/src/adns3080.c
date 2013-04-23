#include "adns3080.h"
#include "adns3080_srom.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <errno.h>

static int adns3080_load_srom(int fd);

int adns3080_init(const char *device)
{
  uint8_t mode = SPI_MODE_3;
  uint8_t bits = 8;
  uint32_t speed = 100000;

	int fd = open(device, O_RDWR);
	if (fd < 0)
  {
		fprintf(stderr, "can't open device\n");
    return -1;
  }

	/*
	 * spi mode
	 */
	int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
  {
		fprintf(stderr, "can't set spi mode\n");
    return -1;
  }

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
  {
    fprintf(stderr, "can't get spi mode\n");
    return -1;
  }

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
  {
		fprintf(stderr, "can't set bits per word\n");
    return -1;
  }

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
  {
		fprintf(stderr, "can't get bits per word\n");
    return -1;
  }

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
  {
		fprintf(stderr, "can't set max speed hz\n");
    return -1;
  }

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
  {
	  fprintf(stderr, "can't get max speed hz\n");
    return -1;
  }

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	while(1)
	{
		uint8_t id = adns3080_spi_write(fd, 0x00, 0x00);

		printf("Got ID: %X\n", id);
		if(id == 0x17)
		{
			break;
		}
		else
		{
			fprintf(stderr, "Cannot detect ADNS3080\n");
			//return -1;
		}
		usleep(100000);
	}
  adns3080_spi_write(fd, 0x8a, 0x10); // Resolution: 1600 cpi
  usleep(100);
  // Activate SROM load mode
  adns3080_spi_write(fd, 0xa0, 0x44);
  adns3080_spi_write(fd, 0xa3, 0x07);
  adns3080_spi_write(fd, 0xa4, 0x88);
  usleep(1000);
  adns3080_spi_write(fd, 0x94, 0x18);
  adns3080_load_srom(fd);
  usleep(1000);
  uint8_t srom_id = adns3080_spi_write(fd, 0x1f, 0x00);
  fprintf(stderr, "SROM_ID: %X\n", srom_id);
  return fd;
}

uint8_t adns3080_spi_write(int fd, uint8_t address, uint8_t value)
{
  uint8_t buf[2];
  buf[0] = address;
  buf[1] = value;

	struct spi_ioc_transfer tr = {
		.tx_buf = (uint64_t)buf,
		.rx_buf = (uint64_t)buf,
		.len = 2,
    .delay_usecs = 75,
	};

	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		fprintf(stderr, "can't send spi message\n");

  return buf[1];
}

struct adns3080_output adns3080_read_motion_burst(int fd)
{
  uint8_t buf[8];

  buf[0] = 0x50;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[5] = 0x00;
  buf[6] = 0x00;
  buf[6] = 0x00;
  buf[7] = 0x00;
	struct spi_ioc_transfer tr = {
		.tx_buf = (uint64_t)buf,
		.rx_buf = (uint64_t)buf,
		.len = 8,
    .delay_usecs = 75,
	};

	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		fprintf(stderr, "can't send spi message\n");

  struct adns3080_output output;
  output.motion = buf[1];
  output.delta_x = buf[2];
  output.delta_y = buf[3];
  output.squal = buf[4];
  output.shutter_upper = buf[5];
  output.shutter_lower = buf[6];
  output.max_pixel = buf[7];

  return output;
}

// buf has to be of size > 900 bytes
int adns3080_read_frame(int fd, uint8_t *buf)
{
  adns3080_spi_write(fd, 0x93, 0x83);
  usleep(2000);

  uint8_t frame[901];
  frame[0] = 0x40;
  struct spi_ioc_transfer tr = {
    .tx_buf = (uint64_t)frame,
    .rx_buf = (uint64_t)frame,
    .len = 901,
    .delay_usecs = 50,
  };

	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
  {
    int error = errno;
    perror("adns3080_read_frame, can't send spi message");
    fprintf(stderr, "ret: %d, errno: %d\n", ret, error);
    //return -1;
  }

  if((frame[1] & 0x40) != 0x40) // First pixel has bit 6 set
  {
    fprintf(stderr, "Fist byte of frame not as expected: %X\n", frame[1]);
    return -1;
  }

  int i;
  for(i = 0; i < 899; i++)
  {
    buf[i] = (frame[900-i] & 0x7F); // All other pixels have MSB set to 1
  }
  buf[899] = frame[1];
  return 0;
}

static int adns3080_load_srom(int fd)
{
  struct spi_ioc_transfer tr = {
    .tx_buf = (uint64_t)srom_contents,
    .len = sizeof(srom_contents),
    .delay_usecs = 50,
  };

	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
  {
    perror("adns3080_load_srom, can't send spi message");
    return -1;
  }
  return 0;
}
