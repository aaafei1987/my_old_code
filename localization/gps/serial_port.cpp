#include "serial_port.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>


SerialPort::SerialPort(const std::string &portname)
    : port_(portname), fd_(-1), is_open_(false)
{
}

bool SerialPort::open(int baudrate, char databit, char stopbit, char parity)
{
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd_ < 0) {
        std::cout << port_ << " open failed , may be you need 'sudo' permission." << std::endl;
        return false;
    }

    if(setPara(baudrate, databit,stopbit,parity) < 0)
        return false;
    is_open_ = true;
    return true;
}

int SerialPort::getBaudRate( int baudrate)
{
    switch(baudrate)
    {
    case 2400:
        return (B2400);
    case 4800:
        return (B4800);
    case 9600:
        return (B9600);
    case 19200:
        return (B19200);
    case 38400:
        return (B38400);
    case 57600:
        return (B57600);
    case 115200:
        return (B115200);
    case 230400:
	return (B230400);
    case 460800:
	return (B460800);
    default:
        std::cout << "Unsupported BaudRate:" << baudrate << std::endl;
        return (B115200);
    }
}

int SerialPort::setPara(int baudrate, char databit, char stopbit, char parity)
{
    struct termios termios_new;
    bzero( &termios_new, sizeof(termios_new));
    cfmakeraw(&termios_new);//就是将终端设置为原始模式
    termios_new.c_cflag = getBaudRate(baudrate);
    termios_new.c_cflag |= CLOCAL | CREAD;
  //  termios_new.c_iflag = IGNPAR | IGNBRK;

    termios_new.c_cflag &= ~CSIZE;
    switch (databit)
    {
    case 0:
        termios_new.c_cflag |= CS5;
        break;
    case 1:
        termios_new.c_cflag |= CS6;
        break;
    case 2:
        termios_new.c_cflag |= CS7;
        break;
    case 3:
        termios_new.c_cflag |= CS8;
        break;
    default:
        termios_new.c_cflag |= CS8;
        break;
    }

    switch (parity)
    {
    case 0:  				//as no parity
        termios_new.c_cflag &= ~PARENB;    //Clear parity enable
      //  termios_new.c_iflag &= ~INPCK; /* Enable parity checking */  //add by fu
        break;
    case 1:
        termios_new.c_cflag |= PARENB;     // Enable parity
        termios_new.c_cflag &= ~PARODD;
        break;
    case 2:
        termios_new.c_cflag |= PARENB;
        termios_new.c_cflag |= ~PARODD;
        break;
    default:
        termios_new.c_cflag &= ~PARENB;   // Clear parity enable
        break;
    }
    switch (stopbit)// set Stop Bit
    {
    case 1:
        termios_new.c_cflag &= ~CSTOPB;
        break;
    case 2:
        termios_new.c_cflag |= CSTOPB;
        break;
    default:
        termios_new.c_cflag &= ~CSTOPB;
        break;
    }

    tcflush(fd_,TCIFLUSH);  
    tcflush(fd_,TCOFLUSH);  
    termios_new.c_cc[VTIME] = 1;    
    termios_new.c_cc[VMIN] = 1; 
    tcflush (fd_, TCIFLUSH);
    return tcsetattr(fd_,TCSANOW,&termios_new);
}

int SerialPort::writeData(const char *data, int datalength)
{
    if(fd_ < 0) { return -1;}

    int len = 0, total_len = 0;
    for (total_len=0 ; total_len < datalength;)
    {
        len = 0;
        len = write(fd_, &data[total_len], datalength - total_len);
        if (len > 0)
        {
            total_len += len;
        }
        else if(len <= 0)
        {
            len = -1;
            break;
        }
    }
    return len;
}

int SerialPort::readData(void *data, int datalength) {
    if (fd_ < 0)
    {
        return -1;
    }
    int len = 0;
    memset(data, 0, datalength);

    int max_fd = 0;
    fd_set readset ={0};
    struct timeval tv ={0};

    FD_ZERO(&readset);
    FD_SET((unsigned int)fd_, &readset);
    max_fd = fd_ +1;
    tv.tv_sec=0;
    tv.tv_usec=1000;
    if (select(max_fd, &readset, NULL, NULL,&tv ) < 0)
    {
        std::cout << "ReadData: select error." << std::endl;
        return -1;
    }
    int nRet = FD_ISSET(fd_, &readset);
    if (nRet)
    {
        len = read(fd_, data, datalength);
    }
    return len;
}

bool SerialPort::isOpen()
{
	return fd_>0 ? true:false;
}

void SerialPort::close() {
    struct termios termios_old;
    if(fd_ > 0)
    {
        tcsetattr (fd_, TCSADRAIN, &termios_old);
        ::close(fd_);
		is_open_ = false;
    }
}


