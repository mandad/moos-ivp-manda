/************************************************************/
/*    NAME: Mike Bogochow, Jeff Masucci, Cody Noel          */
/*    ORGN: UNH                                             */
/*    FILE: NetComm.cpp                                     */
/*    DATE: April 2014                                      */
/************************************************************/
/* This is an abstract implementation of IMOOSComm for      */
/* communicating over a network.                            */
/************************************************************/

#include "NetComm.h"
#include "NetUtil/tcpblockio.h"

#ifdef WIN32

#else
#include <unistd.h>
#endif

#include <fcntl.h>
#include <sys/stat.h>
#include <sstream>
#include <string.h>

using namespace std;

bool NetClientComm::writeMsg(const char *name, double value)
{
  ostringstream ss;
  ss << value;
  return writeMsg(name, ss.str().c_str());
}

bool NetClientComm::writeMsg(const char *name, const char *value)
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
//!!!! block or not? !!!!
  // Write the data to the socket
  nBytes = writeblock(socketFD, (void *)msg, msgLen);
  if (nBytes < 0)
    return false; // "Error writing to frontseat"

  return true;
}

int NetClientComm::readMsg(char *buffer, int bufferLen)
{
  return read(socketFD, buffer, bufferLen); // BLOCK ?
}
