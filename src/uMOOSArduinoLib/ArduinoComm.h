/************************************************************/
/*    NAME: Mike Bogochow, Jeff Masucci, Cody Noel          */
/*    ORGN: UNH                                             */
/*    FILE: ARDUINOComm.h                                   */
/*    DATE: March 2014                                      */
/************************************************************/
/* This is an implementation of IMOOSComm for communicating */
/* with an Arduino.                                         */
/************************************************************/

#ifndef ARDUINOCOMM_H
#define ARDUINOCOMM_H

#include "IMOOSComm.h"

class ArduinoComm : public IMOOSComm
{
public:
  ArduinoComm(const char *portName, int baudRate, const char *delimiter);
  ~ArduinoComm();

  bool openComm();
  bool writeMsg(const char *name, double value);
  bool writeMsg(const char *name, const char *value);
  int readMsg(char *buffer, int bufferLen);

protected:
  int serialFD;

  const char *delim;
  const char *serialPortName;
  const int baudRate;
  //const int BUF_SIZE = 4096;
};

#endif /* ARDUINOCOMM_H */

