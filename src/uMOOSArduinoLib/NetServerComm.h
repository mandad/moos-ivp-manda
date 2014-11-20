/************************************************************/
/*    NAME: Mike Bogochow, Jeff Masucci, Cody Noel          */
/*    ORGN: UNH                                             */
/*    FILE: NetServerComm.h                                 */
/*    DATE: April 2014                                      */
/************************************************************/
/* This is an implementation of IMOOSComm for communicating */
/* over a network as a server.                              */
/************************************************************/

#ifndef NETSERVERCOMM_H
#define NETSERVERCOMM_H

#include "NetComm.h"

class NetServerComm : public NetComm
{
public:
  NetServerComm(char *listenPort, char *listenName, const char *delimiter);
  ~NetServerComm();

  bool openComm();
  
protected:
  char *listenPort;
  char *listenName;
  struct sockaddr *listenAddress;
};

#endif /* NETSERVERCOMM_H */

