#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string.h>
#include <string>

class SerialPort
{
public:
    explicit SerialPort(const std::string& portname);
    bool open(int baudrate, char databit=8, char stopbit=1, char parity=0);
    int setPara(int baudrate, char databit, char stopbit, char parity);
    bool isOpen() const;
   

    int writeData(const char *data, int datalength);
    int readData(void *data, int length);

	bool isOpen();
    void close();

	
    int getBaudRate(int baudrate);
private:
    std::string port_;
    int fd_;
	bool is_open_;
};

#endif // SERIALPORT_H

