#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>
#include <sys/types.h>
#include <termios.h>


class SerialPort {
public:
  enum OpenMode
  {
    WriteOnly,
    ReadOnly,
    ReadWrite
  };

  enum BaudRate
  {
    Baud1200 = B1200,
    Baud2400 = B2400,
    Baud4800 = B4800,
    Baud9600 = B9600,
    Baud19200 = B19200,
    Baud38400 = B38400,
    Baud57600 = B57600,
    Baud115200 = B115200,
    UnknownBaud = -1
  };

  enum DataBits
  {
    Data5 = 5,
    Data6 = 6,
    Data7 = 7,
    Data8 = 8,
    UnknownDataBits = -1
  };

  enum Parity
  {
    NoParity = 0,
    EvenParity = 2,
    OddParity = 3,
    SpaceParity = 4,
    MarkParity = 5,
    UnknownParity = -1
  };

  enum StopBits
  {
    OneStop = 1,
    OneAndHalfStop = 3,
    TwoStop = 2,
    UnknownStopBits = -1
  };

  enum FlowControl
  {
    NoFlowControl,
    HardwareControl,
    SoftwareControl,
    UnknownFlowControl = -1
  };
  SerialPort();
  ~SerialPort();

  bool open(const std::string portName,
            OpenMode mode,
            bool nonBlock = false,
            BaudRate baudrate = Baud9600,
            DataBits dataBits = Data8,
            StopBits stopBits = OneStop,
            Parity parity = NoParity,
            FlowControl flowControl = NoFlowControl);



  void close();

  int fd()
  { return fd_;}

private:
  bool initialize(bool nonBlock,
                  BaudRate baudrate ,
                  DataBits dataBits ,
                  StopBits stopBits ,
                  Parity parity ,
                  FlowControl flowControl);

  int fd_;
  termios restoredTermios;

};

#endif // SERIALPORT_H
