#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h> 
#include <sys/time.h>
#include <sys/types.h>
#include "serialport.h"
#include <string.h>

#define TIMEOUT 10

//open serial port
int serial_open(char* device)
{
	int fd;
	fd = open(device, O_RDWR | O_NOCTTY);
	return fd;
}

//init serial port baudrate
uint8_t serial_init(int fd,uint32_t baudrate)
{
	uint8_t status=0;
	struct termios SerialPortSettings;
	
	tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	switch(baudrate) {
		case 4800:
		cfsetispeed(&SerialPortSettings,B4800); 
		cfsetospeed(&SerialPortSettings,B4800); 
		break;
		case 9600:
		cfsetispeed(&SerialPortSettings,B9600); 
		cfsetospeed(&SerialPortSettings,B9600); 
		break;
		case 19200:
		cfsetispeed(&SerialPortSettings,B19200); 
		cfsetospeed(&SerialPortSettings,B19200); 
		break;
		case 38400:
		cfsetispeed(&SerialPortSettings,B38400); 
		cfsetospeed(&SerialPortSettings,B38400); 
		break;
		case 57600:
		cfsetispeed(&SerialPortSettings,B57600); 
		cfsetospeed(&SerialPortSettings,B57600); 
		break;
		case 115200:
		cfsetispeed(&SerialPortSettings,B115200); 
		cfsetospeed(&SerialPortSettings,B115200); 
		break;
		case 576000:
		cfsetispeed(&SerialPortSettings,B576000); 
		cfsetospeed(&SerialPortSettings,B576000); 
		break;
		case 921600:
		cfsetispeed(&SerialPortSettings,B921600); 
		cfsetospeed(&SerialPortSettings,B921600); 
		break;
		default:
		#ifdef serial_debug
		printf("baudrate not support\r\n");
		#endif
		status = 1;
		return status;
		break;
	}
	/* 8N1 Mode */
	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
	
	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
	
	
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

	SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/
	
	/* Setting Time outs */
	SerialPortSettings.c_cc[VMIN] = 10; /* Read at least 10 characters */
	SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */


	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0){
		status = 1;
		#ifdef serial_debug
		printf("\n  ERROR ! in Setting attributes");
		#endif
	}else{
		#ifdef serial_debug
		printf("init baudrate:%d\r\n",baudrate);
		#endif
	}
	tcflush(fd, TCIFLUSH);

	return status;
}

//read data from serial port
int serial_read(int fd, uint8_t* buf,uint16_t len)
{
	int temp_len;
	
	#ifdef serial_debug
	printf("rlen:%d\r\n",len);
	#endif
	
	temp_len=read(fd,buf,len);
	return temp_len;
}

//write data to serial port
int serial_write(int fd, uint8_t* buf,uint16_t len)
{
	int temp_len;
	#ifdef serial_debug
	printf("wlen:%d\r\n",len);
	#endif
	temp_len = write(fd,buf,len);
	return temp_len;
}


//close serial port
void serial_close(int fd)
{
	close(fd);
}


int main(int argc, char* argv[])
{
	
	int fd,ret;
	uint8_t read_buffer[32]; 
	uint8_t write_buffer[32];
	
	uint8_t i;
	int wr_len;
	
	struct timeval tv;
	fd_set readfds;
	

	if(argc<2){
		printf("usage: %s serial_device\r\n",argv[0]);
		return 1;
	}

	//open serial device
	fd = serial_open(argv[1]);
	if(fd<0){
		printf("open serial device erro!\r\n");
		return 1;
	}else{
		printf("open device:%s\r\n",argv[1]);
	}
	
	//init baudrate 115200
	serial_init(fd,115200);
	
	//set write buf
	printf("write data:\r\n");
	for(i=0;i<32;i++){
		write_buffer[i]=0x30+i;
		printf("%c ",write_buffer[i]);
	}
	printf("\r\n");
	wr_len=serial_write(fd,write_buffer,sizeof(write_buffer));
	printf("serial write length:%d\r\n",wr_len);

	#if 1
	//select
	FD_ZERO(&readfds);
	FD_SET(fd, &readfds);
	tv.tv_sec = TIMEOUT;
	tv.tv_usec = 0;
	ret = select((fd+1),&readfds,NULL,NULL,&tv);
	if(ret<0){
		printf("select erro!\r\n");
	}else if(ret==0){
		printf("timeout!\r\n");
	}else{
		if(FD_ISSET(fd, &readfds)) {
			wr_len = serial_read(fd, read_buffer,sizeof(read_buffer));
			if(wr_len == -1) {
				printf("read erro!\r\n");
				return 1;
			}
			if(wr_len) {
				printf("len:%d,read: %s\n", wr_len,read_buffer);
			}
		}
	}
	#else
	wr_len = serial_read(fd, read_buffer,sizeof(read_buffer));
	printf("len:%d,read: %s\n", wr_len,read_buffer);
	#endif
	serial_close(fd); 

	return 0;
}

