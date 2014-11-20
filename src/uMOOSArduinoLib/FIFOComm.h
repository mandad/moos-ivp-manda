/************************************************************/
/*    NAME: Mike Bogochow                                   */
/*    ORGN: UNH                                             */
/*    FILE: FIFOComm.h                                     */
/*    DATE: Mar, 2014                                       */
/************************************************************/
/* This is an implementation of IMOOSComm using Linux FIFOs.  It is primarily for testing purposes but could be used to pass messages to a real frontend if it is located on the same physical machine as the MOOS Community.         */
/************************************************************/

#ifndef FIFOCOMM_H
#define FIFOCOMM_H

#include "IMOOSComm.h"

class FIFOComm : public IMOOSComm
{
public:
  FIFOComm(const char *readFIFOName, const char *writeFIFOName,
    const char *delimiter);
  ~FIFOComm();

  bool openComm();
  bool writeMsg(const char *name, double value);
  bool writeMsg(const char *name, const char *value);
  int readMsg(char *buffer, int bufferLen);

protected:
  int rd_fifo;
  int wr_fifo;

  const char *delim;
  const char *READ_FIFO_NAME;
  const char *WRITE_FIFO_NAME;
  //const int BUF_SIZE = 4096;
};

#endif /* FIFOCOMM_H */

