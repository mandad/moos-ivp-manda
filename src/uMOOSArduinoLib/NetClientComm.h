/************************************************************/
/*    NAME: Mike Bogochow, Jeff Masucci, Cody Noel          */
/*    ORGN: UNH                                             */
/*    FILE: NetClientComm.h                                 */
/*    DATE: April 2014                                      */
/************************************************************/
/* This is an implementation of IMOOSComm for communicating */
/* over a network as a client.                              */
/************************************************************/

#ifndef NETCLIENTCOMM_H
#define NETCLIENTCOMM_H

#include "NetComm.h"

class NetClientComm : public NetComm
{
public:
  NetClientComm(char *serverPort, char *serverNode, const char *delimiter);
  ~NetClientComm();

  bool openComm();

protected:
  struct sockaddr *serverIP;
  char *serverPort;
  char *serverNode;
  struct sockaddr *clientIP;
};

#endif /* NETCLIENTCOMM_H */

