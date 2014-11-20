/************************************************************/
/*    NAME: Mike Bogochow, Jeff Masucci, Cody Noel          */
/*    ORGN: UNH                                             */
/*    FILE: NetClientComm.cpp                               */
/*    DATE: April 2014                                      */
/************************************************************/
/* This is an implementation of IMOOSComm for communicating */
/* over a network as a client.                              */
/************************************************************/

#include "NetClientComm.h"
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

/**
 * Create a new client communications object.
 *
 * @param serverPort
 *      the port number the server is running on
 * @param serverNode 
 *      either an IPv4 address or a DNS name for the server
 * @param delimiter
 *      the delimiter character for messages between client and server
 */
NetClientComm::NetClientComm(char *serverPort, char *serverNode, 
  const char *delimiter)
  //: delim(delimiter)
{
  this->delim = delimiter;
  this->serverPort = serverPort;
  this->serverNode = serverNode;
  socketFD = -1;
}

NetClientComm::~NetClientComm()
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
bool NetClientComm::openComm()
{
  no_sigpipe();
  socketFD = openclient(serverPort, serverNode, serverIP, clientIP);
  return socketFD >= 0;
}

