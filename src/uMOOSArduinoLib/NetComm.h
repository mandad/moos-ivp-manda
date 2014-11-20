/************************************************************/
/*    NAME: Mike Bogochow, Jeff Masucci, Cody Noel          */
/*    ORGN: UNH                                             */
/*    FILE: NetComm.h                                       */
/*    DATE: April 2014                                      */
/************************************************************/
/* This is an abstract implementation of IMOOSComm for      */
/* communicating over a network.                            */
/************************************************************/

#ifndef NETCOMM_H
#define NETCOMM_H

#include "IMOOSComm.h"

class NetComm : public IMOOSComm
{
public:
  virtual ~NetComm() {};

  virtual bool openComm()=0;
  bool writeMsg(const char *name, double value);
  bool writeMsg(const char *name, const char *value);
  int readMsg(char *buffer, int bufferLen);

protected:
  int socketFD;

  const char *delim;
  //const int BUF_SIZE = 4096;
};

#endif /* NETCOMM_H */

