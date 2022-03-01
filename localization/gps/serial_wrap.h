/**************************************************************
 ***** Copyright (c) 2020 Fujian(Quanzhou)-HIT Research Institute
 ***** of Engineering and Technology All rights reserved.
 **************************************************************/

#ifndef __SERIAL_WRAP_H__
#define __SERIAL_WRAP_H__

#include "serial_port.h"
#include <string>

#define MAX_BUFF_LEN 160

class SerialWrap
{
public:
    SerialWrap(std::string& port, int baudrate=38400, char databit=8, char stopbit=1, char parity=0);
    ~SerialWrap();

	int openSerial();
	int readMsg(std::string& msg);
	int writeMsg(std::string& msg);
	
private:
    SerialPort * serial_;
	std::string port_;
	int baudrate_;
	char databit_;
	char stopbit_;
	char parity_;
    int fd_;
    unsigned char recv_data_[MAX_BUFF_LEN];
	int recv_len_;
};


#endif
