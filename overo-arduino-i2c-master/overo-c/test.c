/*
	 Simple I2C communication test with an Arduino as the slave device.
 */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

#define ARDUINO_I2C_ADDRESS 0x10

int main(int argc, char **argv)
{
	int fh;
	char buff[4];
	int len, sent, rcvd;

	fh = open("/dev/i2c-3", O_RDWR);

	if (fh < 0) {
		perror("open");
		return 1;
	}

	ioctl(fh, I2C_SLAVE, ARDUINO_I2C_ADDRESS);

	while(1)
	{

		rcvd = read(fh, buff, 4);
		printf("%i\n",rcvd);
		if (rcvd > 0)
		{
			printf("get data: %d \n",(unsigned int)buff[0]);

		}
  usleep(10000);
	}
	close(fh);

	return 0;
}
