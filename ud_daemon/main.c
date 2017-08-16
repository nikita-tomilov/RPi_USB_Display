#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>


char *portname = "/dev/ttyACM0";
char *dump_path = "/tmp/fbdump.bin";

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

#define MASK_MILD 0xE79C
#define MASK_STRONG 0xF7DE
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3); }
int check_same_color(uint16_t a, uint16_t b, uint16_t mask) {
    return ((a & mask) == (b & mask));
}
int check_same_color_2(uint8_t a1, uint8_t a2, uint8_t b1, uint8_t b2, uint16_t mask) {
    uint16_t a = ((uint16_t)(a1) << 8) | a2;
    uint16_t b = ((uint16_t)(b1) << 8) | b2;
    return ((a & mask) == (b & mask));
}

int bytes_per_write = 24;
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

#define PIXELS_PER_CHUNK 8
#define BYTES_PER_CHUNK PIXELS_PER_CHUNK * 2

#define CHUNKS_PER_WRITE 4
#define BYTES_PER_WRITE (4 + BYTES_PER_CHUNK) * CHUNKS_PER_WRITE

uint8_t oldbuf[320 * 240 * 2];
uint8_t buf[320 * 240 * 2];
uint8_t flashbuf[320 * 240 * 6];

void send_raw_withxy(int fd, char* path) {
	//printf("Preprocessing...\n");
	
	memcpy(oldbuf, buf, 320 * 240 * 2);

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
	
	long offset = 0;

	int x = 0, y = 0;
	int has_diff = 0;
	

	for (int i = 0; i < sizeof(buf); i += BYTES_PER_CHUNK) {

	    for (int j = 0; j < BYTES_PER_CHUNK; j+=2) {
		if (!check_same_color_2(buf[i + j], buf[i + j + 1], oldbuf[i + j], oldbuf[i + j + 1], MASK_MILD)) {
		    has_diff = 1;
		    break;
		}
	    }

	    if (has_diff) {
			flashbuf[offset] = (uint8_t)((x >> 8) & 0xFF); offset++;
			flashbuf[offset] = (uint8_t)((x) & 0xFF); offset++;
			flashbuf[offset] = (uint8_t)((y >> 8) & 0xFF); offset++;
			flashbuf[offset] = (uint8_t)((y) & 0xFF); offset++;

			memcpy(flashbuf + offset, buf + i, BYTES_PER_CHUNK);
			offset += BYTES_PER_CHUNK;

	    }
	    has_diff = 0;
	    
	    x+=PIXELS_PER_CHUNK;
	    if (x >= 320) { y++; x = 0; }
	    if (y >= 240) { y = 0; x = 0; }
	}

	long count = offset;
	offset = 0;
	
	
	for (int i = 0; i < count; i += BYTES_PER_WRITE) {
	    write(fd, flashbuf + i, BYTES_PER_WRITE); 
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
	
	set_interface_attribs (fd, B921600, /*B230400,*/ 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking
	
	sleep(3); //wait for arduino to get ready

	while (1) {
		send_raw_withxy(fd, dump_path);
		usleep(50 * 1000); //to avoid arduino hiccups
	}

	return 0;
}

