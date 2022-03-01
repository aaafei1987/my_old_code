/**************************************************************
 ***** Copyright (c) 2019 Fujian(Quanzhou)-HIT Research Institute
 ***** of Engineering and Technology All rights reserved.
 **************************************************************/
#include "serial_wrap.h"
#include <iostream>
using namespace std;

SerialWrap::SerialWrap(std::string& port, int baudrate, char databit, char stopbit, char parity) :
	port_(port)
	,serial_(nullptr)
	,baudrate_(baudrate)
    ,databit_(databit)
	,stopbit_(stopbit)
	,parity_(parity)
{
	serial_ = new SerialPort(port);
	recv_len_ = 0;
	memset(recv_data_, 0, sizeof(recv_data_));
}

SerialWrap::~SerialWrap()
{
	if (serial_->isOpen())
	{
		serial_->close();
	}
}

int SerialWrap::openSerial()
{
	if (serial_ != nullptr)
	{
		return serial_->open(baudrate_, databit_, stopbit_, parity_);
	}
	return false;
}

int SerialWrap::writeMsg(std::string& msg)
{
	return serial_->writeData(msg.c_str(), msg.size());
}

int SerialWrap::readMsg(std::string& msg)
{
	char buff[16];
	int dataLen = 0;
	static bool msgStarted = false;
	char* p = nullptr;
	char* q = nullptr;

	dataLen = serial_->readData(buff,16);
	if (dataLen <= 0)
	{	
		return -1;
	}

	if ( msgStarted == false)
	{
		if ( nullptr != (p = strchr(buff, '$')))
		{
			memcpy(recv_data_, p, dataLen - (p - buff));
			recv_len_ += (dataLen - (p - buff));
			msgStarted = true;
		}
	}
	else if ( nullptr != ( p = strchr(buff, '\r')))
	{
		memcpy(recv_data_ + recv_len_, buff, p - buff);
		recv_len_ +=  p - buff;
		recv_data_[recv_len_] = 0;
		msg = (char*)recv_data_;
		msgStarted = false;
		recv_len_ = 0;

		if (nullptr != ( q = strchr(p, '$')))
		{
			memcpy(recv_data_, q, dataLen - (q - buff));
			recv_len_ = dataLen - (q - buff);
			msgStarted = true;
		} 
		
		return 0;
	}
	else
	{
		memcpy(recv_data_ + recv_len_, buff, dataLen);
		recv_len_ += dataLen;
	}

	return -1;
}
