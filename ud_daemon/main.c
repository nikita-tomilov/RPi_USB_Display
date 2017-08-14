#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>


char *portname = "/dev/ttyACM0";
char *dump_path = "/home/pi/dump.bin";
#define bytes_per_write 320

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                
                return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

       
}

uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3); }


void send_raw(int fd, char* path) {
	uint8_t buf[320 * 240 * 2];
	//printf("Preprocessing...\n");
	
	FILE* input = fopen(path, "rb");
	fread(buf, sizeof(buf), 1, input);
	fclose(input);
	
	uint8_t tmp;
	
	for (int i = 0; i < sizeof(buf); i+= 2) {
		//byte order
		tmp = buf[i];
		buf[i] = buf[i + 1];
		buf[i + 1] = tmp;
	}
	
	//printf("Flashing...\n");
	for (int i = 0; i < sizeof(buf); i += bytes_per_write) {
		write(fd, buf + i, bytes_per_write); 
	}
}

int main(int argc, char **argv)
{
	
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	
	if (fd < 0) {
		printf("Open error %d\n", fd);
		return -1;
	}
	
	printf("Using %s\n", portname);
	
	set_interface_attribs (fd, B1000000, /*B230400,*/ 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking
	
	sleep(3); //wait for arduino to get ready

	while (1) {
		send_raw(fd, dump_path);
		usleep(50 * 1000); //to avoid arduino hiccups
	}

	return 0;
}

