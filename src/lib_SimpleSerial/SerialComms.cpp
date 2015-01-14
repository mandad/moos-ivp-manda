/*
 * SerialComms.cpp
 *
 *  Created on:   July 20, 2014
 *      Author:   Alon Yaari
 *      INSPIRED BY:
 *          arduino-serial-lib -- simple library for reading/writing serial ports
 *          2006-2013, Tod E. Kurt, http://todbot.com/blog/
 *
 */

#include "SerialComms.h"

#include <iostream>    // Standard input/output definitions
#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <errno.h>    // Error number definitions
#include <termios.h>  // POSIX terminal control definitions
//#include <string.h>   // String function definitions
#include <sys/ioctl.h>

using namespace std;

bool SerialComms::dispatch(void * pParam)
{
  SerialComms* pMe = (SerialComms*)pParam;
  return pMe->SerialLoop();
}

bool SerialComms::Run()
{
  serialThread.Initialise(dispatch, this);
  return serialThread.Start();
}

bool SerialComms::SerialLoop()
{
  if (!bGoodSerialComms)
    return false;

  // SerialIncoming()
  //      - Accumulates characters coming in on the serial port
  //      - Returns FALSE if serial port fails (read error or timeout)s
  //      - Returns TRUE if bQuit gets set to true
  //      - Otherwise runs continuously
  return SerialIncoming(lastErrorMsg);
}

SerialComms::SerialComms(string port, int baud, string& errMsg)
{
  bQuit            = false;
  fd               = 0;
  m_char_beg       = '$';
  m_char_end       = '\n';
  m_ignore         = "\r";
  bGoodSerialComms = serialport_init(port.c_str(), baud, errMsg);
}

void SerialComms::Set_Delims(char beg, char end, string ignore)
{
  m_char_beg    = beg;
  m_char_end    = end;
  m_ignore      = ignore;
}

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
bool SerialComms::serialport_init(const char* serialport, int baud, string& errMsg)
{
  struct termios toptions;

  //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
  fd = open(serialport, O_RDWR | O_NONBLOCK );

  if (fd == -1) {
    errMsg = strerror(errno);
    return false; }

  //int iflags = TIOCM_DTR;
  //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
  //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

  if (tcgetattr(fd, &toptions) < 0) {
    errMsg = strerror(errno);
    return false; }

  speed_t brate;
  switch (baud) {
    case 4800:   brate = B4800;   break;
    case 9600:   brate = B9600;   break;
    case 19200:  brate = B19200;  break;
    case 38400:  brate = B38400;  break;
    case 57600:  brate = B57600;  break;
    case 115200: brate = B115200; break;
    default:
      errMsg = "Unsupported baud rate.";
      return false; }
  cfsetispeed(&toptions, brate);
  cfsetospeed(&toptions, brate);

  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  toptions.c_cflag &= ~CRTSCTS;       // no flow control
  //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset
  toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  toptions.c_oflag &= ~OPOST; // make raw

  // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  toptions.c_cc[VMIN]  = 0;
  toptions.c_cc[VTIME] = 0;
  //toptions.c_cc[VTIME] = 20;

  tcsetattr(fd, TCSANOW, &toptions);
  if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
    errMsg = strerror(errno);
    return false; }

  errMsg = "";
  return true;
}

int SerialComms::serialport_close()
{
  return close(fd);
}

int SerialComms::serialport_writebyte(uint8_t b, string& errMsg)
{
  int n = write(fd, &b, 1);
  if (n != 1) {
    errMsg = strerror(errno);
    bGoodSerialComms = false;
    return -1; }
  errMsg = "";
return 0;
}

int SerialComms::serialport_write(const char* str, string& errMsg)
{
  int len = strlen(str);
  int n = write(fd, str, len);
  if (n != len) {
    errMsg = strerror(errno);
    bGoodSerialComms = false;
    return -1; }
return 0;
}

bool SerialComms::WriteToSerialPort(string str)
{
  if (str.empty())
    return true;
  string err = "";
  int good = serialport_write(str.c_str(), err);
  return (good == 0);
}

bool SerialComms::SerialIncoming(string& errMsg)
{
  if (!bGoodSerialComms) {
    errMsg = "Cannot listen to serial, port is closed.";
    return false; }
  char b[1];  // read expects an array, so we give it a 1-byte array
  int timeout = TIMEOUT;
  string inLine = "";
  bool inSentence = false;
  while (!bQuit) {
    if (timeout <= 0) {
      errMsg = "Serial port timed out, assuming bad port.";
      return false; }

    int n = read(fd, b, 1);             // Read one character at a time, accessed with b[0]
    switch (n) {

      // Failure to read from the serial port
      case -1:
        errMsg = strerror(errno);
        bGoodSerialComms = false;
        return false;
        break;

      // No bytes to read
      case 0:
        usleep(1 * 1000);               // Wait 1ms (1000 micoseconds) before continuing
        timeout--;
        break;

      // Incoming byte
      default:
        timeout = TIMEOUT;
        if (b[0] == m_char_beg) {       // Start of a data packet
          inSentence = true;
          inLine     = ""; }
        if (inSentence) {
          if (b[0] == m_char_end) {     // End the data packet
            AddToProcessingQueue(inLine);
            inSentence = false; }
          else

            // Reaching here means:
            //    - Good serial comms
            //    - One byte has been read in
            //    - Flagged that we are reading bytes at or after the start delim
            //    - Have not yet reached the end delim
            // Add the character if it's not marked to ignore
            if (m_ignore.find(b[0]) != string::npos)
              inLine += b[0];
          // At some point cut off the sentence
          if (inLine.length() > BUF_SIZE) {
            AddToProcessingQueue(inLine);
            inSentence = false; } }
        break; } }
  return true;
}

// AddToProcessingQueue()
//      - Adds the string to the deque
//      - NOTE: Clears the string in the calling function
void SerialComms::AddToProcessingQueue(string& str)
{
    if (!str.empty()) {
        inLines.push_front(str);
        str = ""; }
}

int SerialComms::DataAvailable()
{
    return inLines.size();
}

string SerialComms::GetNextSentence()
{
    string str = "";
    if (DataAvailable()) {
        str = inLines.back();
        inLines.pop_back(); }
    return str;
}

int SerialComms::serialport_flush()
{
    sleep(2); //required to make flush work, for some reason
    return tcflush(fd, TCIOFLUSH);
}
