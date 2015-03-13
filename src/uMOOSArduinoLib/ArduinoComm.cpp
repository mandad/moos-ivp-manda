/************************************************************/
/*    NAME: Mike Bogochow, Jeff Masucci, Cody Noel          */
/*    ORGN: UNH                                             */
/*    FILE: ARDUINOComm.h                                   */
/*    DATE: March 2014                                      */
/************************************************************/
/* This is an implementation of IMOOSComm for communicating */
/* with an Arduino.                                         */
/************************************************************/

#include "ArduinoComm.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sstream>
#include <string.h>

#include <termios.h>

using namespace std;

ArduinoComm::ArduinoComm(const char *portName, int baudRate, 
  const char *delimiter)
  : serialPortName(portName)
  , baudRate(baudRate)
  , delim(delimiter)
{
}

ArduinoComm::~ArduinoComm()
{
  close(serialFD);
}

/**
 * Open serial port with Arduino.
 * Based on arduino-serial.c by todbot
 * (https://github.com/todbot/arduino-serial)
 *
 * @return true on successful open of the serial port
 *         false if opening the serial port or getting/setting attributes failed
 */
bool ArduinoComm::openComm()
{
  struct termios options;
  speed_t baud = baudRate;

  serialFD = open(serialPortName, O_RDWR | O_NONBLOCK);
  if (serialFD < 0)
  {
    //MOOSTrace("Error opening serial (%s): %s\n", serialPortName,
    //  strerror(errno));
    return false;
  }

  if (tcgetattr(serialFD, &options) < 0)
  {
    //MOOSTrace("Failed to get terminal settings: %s\n", strerror(errno));
    return false;
  }

  switch (baudRate)
  {
    case 4800:   baud=B4800;   break;
    case 9600:   baud=B9600;   break;
#ifdef B14400
    case 14400:  baud=B14400;  break;
#endif
    case 19200:  baud=B19200;  break;
#ifdef B28800
    case 28800:  baud=B28800;  break;
#endif
    case 38400:  baud=B38400;  break;
    case 57600:  baud=B57600;  break;
    case 115200: baud=B115200; break;
  }

  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  // 8N1
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~CRTSCTS;  // no flow control 
  options.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  options.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset
  options.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  options.c_oflag &= ~OPOST; // make raw
  options.c_cc[VMIN]  = 0;
  options.c_cc[VTIME] = 0;
  //toptions.c_cc[VTIME] = 20;

  tcsetattr(serialFD, TCSANOW, &options);
  if (tcsetattr(serialFD, TCSAFLUSH, &options) < 0)
  {
    //MOOSTrace("Failed to set terminal settings: %s\n", strerror(errno));
    return false;
  }

  return true;
}

bool ArduinoComm::writeMsg(const char *name, double value)
{
  ostringstream ss;
  ss << value;
  return writeMsg(name, ss.str().c_str());
}

bool ArduinoComm::writeMsg(const char *name, const char *value)
{
  ostringstream ss;
  ss << name << '=' << value << delim;

  const char *msg = ss.str().c_str(); // Full message to sendi
  size_t msgLen = strlen(msg);        // Length of message we are sending
  ssize_t nBytes;                     // Actual number of bytes written

  // Check if we can write, return if not since we don't want to block
/*  struct pollfd mpollfd = { .fd = fd, .events = POLLOUT };
  MOOSTrace("POLL: %i\n", poll(&mpollfd, 1, 0));
  if (!(poll(&mpollfd, 1, 0) > 0))
    return MOOSFail("Stream is not ready for writing");*/

  // Write to our FIFO
  nBytes = write(serialFD, msg, msgLen);
  if (nBytes < 0)
    return false; // "Error writing to frontseat"

  return true;
}

int ArduinoComm::readMsg(char *buffer, int bufferLen)
{
  return read(serialFD, buffer, bufferLen);
}
