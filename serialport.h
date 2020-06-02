#ifndef __SERIALPORT_H_
#define __SERIALPORT_H_

#include <stdint.h>
#include <stdio.h>

#define serial_debug

//open serial port
int serial_open(char* device);
//init serial port baudrate
uint8_t serial_init(int fd,uint32_t baudrate);
//read data from serial port
int serial_read(int fd, uint8_t* buf,uint16_t len);
//write data to serial port
int serial_write(int fd, uint8_t* buf,uint16_t len);
//close serial port
void serial_close(int fd);


#endif
