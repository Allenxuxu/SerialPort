#include "SerialPort.h"

#include <errno.h>
//#include <fcntl.h>
//#include <sys/ioctl.h>

#include <unistd.h>

#include <sys/stat.h>
#include <fcntl.h>

#include "Log.h"

static inline void set_common_props(termios *tio,bool nonBlock)
{
  ::cfmakeraw(tio);

  tio->c_cflag |= CLOCAL|CREAD;
  tio->c_cc[VTIME] = 0;
  if(nonBlock)
  {
    tio->c_cc[VMIN] = 0;
  }
  else
  {
    tio->c_cc[VMIN] = 1;
  }
}

static bool set_baud_rate(termios *tio,SerialPort::BaudRate baudrate)
{
  //默认收发双发波特率相同
  if ( ::cfsetispeed(tio, baudrate) < 0)
  {
    LOG_SYSERR << "cfsetispeed";
    return false;
  }

  if (::cfsetospeed(tio, baudrate) < 0)
  {
    LOG_SYSERR << "cfsetispeed";
    return false;
  }
  return true;
}

static inline void set_databits(termios *tio, SerialPort::DataBits databits)
{
  tio->c_cflag &= ~CSIZE;
  switch (databits) {
    case SerialPort::Data5:
      tio->c_cflag |= CS5;
      break;
    case SerialPort::Data6:
      tio->c_cflag |= CS6;
      break;
    case SerialPort::Data7:
      tio->c_cflag |= CS7;
      break;
    case SerialPort::Data8:
      tio->c_cflag |= CS8;
      break;
    default:
      tio->c_cflag |= CS8;
      break;
  }
}

static inline void set_parity(termios *tio, SerialPort::Parity parity)
{
  tio->c_iflag &= ~(PARMRK | INPCK);
  tio->c_iflag |= IGNPAR;

  switch (parity) {

#ifdef CMSPAR
    // Here Installation parity only for GNU/Linux where the macro CMSPAR.
    case SerialPort::SpaceParity:
      tio->c_cflag &= ~PARODD;
      tio->c_cflag |= PARENB | CMSPAR;
      break;
    case SerialPort::MarkParity:
      tio->c_cflag |= PARENB | CMSPAR | PARODD;
      break;
#endif
    case SerialPort::NoParity:
      tio->c_cflag &= ~PARENB;
      break;
    case SerialPort::EvenParity:
      tio->c_cflag &= ~PARODD;
      tio->c_cflag |= PARENB;
      break;
    case SerialPort::OddParity:
      tio->c_cflag |= PARENB | PARODD;
      break;
    default:
      tio->c_cflag |= PARENB;
      tio->c_iflag |= PARMRK | INPCK;
      tio->c_iflag &= ~IGNPAR;
      break;
  }
}

static inline void set_stopbits(termios *tio, SerialPort::StopBits stopbits)
{
  switch (stopbits) {
    case SerialPort::OneStop:
      tio->c_cflag &= ~CSTOPB;
      break;
    case SerialPort::TwoStop:
      tio->c_cflag |= CSTOPB;
      break;
    default:
      tio->c_cflag &= ~CSTOPB;
      break;
  }
}

static inline void set_flowcontrol(termios *tio, SerialPort::FlowControl flowcontrol)
{
  switch (flowcontrol) {
    case SerialPort::NoFlowControl:
      tio->c_cflag &= ~CRTSCTS;
      tio->c_iflag &= ~(IXON | IXOFF | IXANY);
      break;
    case SerialPort::HardwareControl:
      tio->c_cflag |= CRTSCTS;
      tio->c_iflag &= ~(IXON | IXOFF | IXANY);
      break;
    case SerialPort::SoftwareControl:
      tio->c_cflag &= ~CRTSCTS;
      tio->c_iflag |= IXON | IXOFF | IXANY;
      break;
    default:
      tio->c_cflag &= ~CRTSCTS;
      tio->c_iflag &= ~(IXON | IXOFF | IXANY);
      break;
  }
}


SerialPort::SerialPort()
  :fd_(-1)
{
}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string portName,
                      SerialPort::OpenMode mode,
                      bool nonBlock,
                      SerialPort::BaudRate baudrate,
                      SerialPort::DataBits dataBits,
                      SerialPort::StopBits stopBits,
                      SerialPort::Parity parity,
                      SerialPort::FlowControl flowControl)
{
  int flags = O_CLOEXEC | O_NOCTTY ;
  if(nonBlock)
  {
    flags |= O_NONBLOCK;
  }

  switch (mode)
  {
    case WriteOnly:
      flags |= O_WRONLY;
      break;
    case ReadOnly:
      flags |= O_RDONLY;
      break;
    case ReadWrite:
      flags |= O_RDWR;
      break;
    default:
      assert(!"no this open mode");
      break;
  }

  fd_ = ::open(portName.c_str(),flags);
  if(fd_ < 0)
  {
    LOG_ERROR <<"open fail : "<<portName;
    return false;
  }

  return initialize(nonBlock, baudrate,dataBits,stopBits,parity,flowControl);
}



bool SerialPort::initialize(bool nonBlock ,
                            SerialPort::BaudRate baudrate,
                            SerialPort::DataBits dataBits,
                            SerialPort::StopBits stopBits,
                            SerialPort::Parity parity,
                            SerialPort::FlowControl flowControl)
{
#ifdef TIOCEXCL
  if (::ioctl(fd_, TIOCEXCL) == -1)
    LOG_SYSERR << "ioctl" ;
#endif

  termios tio;

  ::memset(&tio, 0, sizeof(termios));
  if (::tcgetattr(fd_, &tio) == -1)
  {
    LOG_SYSERR << "tcgetattr";
    return false;
  }

  restoredTermios = tio;

  set_common_props(&tio,nonBlock);
  set_databits(&tio, dataBits);
  set_parity(&tio, parity);
  set_stopbits(&tio, stopBits);
  set_flowcontrol(&tio, flowControl);

  if (::tcsetattr(fd_, TCSANOW, &tio) == -1)
  {
    LOG_SYSERR << "tcsetattr";
    return false;
  }

  if(!set_baud_rate(&tio,baudrate))
  {
    LOG_ERROR << "set baudrate err";
    return false;
  }
  return true;
}

void SerialPort::close()
{
  ::tcsetattr(fd_, TCSANOW, &restoredTermios);

#ifdef TIOCNXCL
  ::ioctl(descriptor, TIOCNXCL);
#endif

  ::close(fd_);
  fd_ = -1;
}

