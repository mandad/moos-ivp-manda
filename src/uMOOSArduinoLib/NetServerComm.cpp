/************************************************************/
/*    NAME: Mike Bogochow, Jeff Masucci, Cody Noel          */
/*    ORGN: UNH                                             */
/*    FILE: NetServerComm.cpp                               */
/*    DATE: April 2014                                      */
/************************************************************/
/* This is an implementation of IMOOSComm for communicating */
/* over a network as a server.                              */
/************************************************************/

#include "NetServerComm.h"
#include "NetUtil/server.h"
#include "NetUtil/tcpblockio.h"
#include "NetUtil/no_sigpipe.h"

#ifdef WIN32

#else
#include <unistd.h>
#endif

#include <fcntl.h>
#include <sys/stat.h>
#include <sstream>
#include <string.h>

using namespace std;

NetServerComm::NetServerComm(char *listenPort, char *listenName,
  const char *delimiter)
{
  this->delim = delimiter;
  this->listenPort = listenPort;
  this->listenName = listenName;
  socketFD = -1;
}

NetServerComm::~NetServerComm()
{
  if (socketFD > -1)
    close(socketFD);
}

/**
 * Open socket.
 *
 * @return true on successful open of socket
 *         false otherwise
 */
bool NetServerComm::openComm()
{
  no_sigpipe();
  socketFD = openlistener(listenPort, listenName, listenAddress);
  return socketFD >= 0;
}

