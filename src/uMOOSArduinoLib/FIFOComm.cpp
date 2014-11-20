/************************************************************/
/*    NAME: Mike Bogochow                                   */
/*    ORGN: UNH                                             */
/*    FILE: FIFOComm.cpp                                    */
/*    DATE: March 2014                                      */
/************************************************************/
/* This is an implementation of IMOOSComm using Linux       */
/* FIFOs.  It is primarily for testing purposes but could   */
/* be used to pass messages to a real frontend if it is     */
/* located on the same physical machine as the MOOS         */
/* Community.                                               */
/************************************************************/

#include "FIFOComm.h"

#include <stdarg.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sstream>
#include <string.h>

using namespace std;
#include <iostream>

FIFOComm::FIFOComm(const char *readFIFOName, const char *writeFIFOName,
  const char *delimiter)
  : READ_FIFO_NAME(readFIFOName),
    WRITE_FIFO_NAME(writeFIFOName),
    delim(delimiter)
{
}

FIFOComm::~FIFOComm()
{
  close(rd_fifo);
  close(wr_fifo);

  unlink(WRITE_FIFO_NAME);
}

bool FIFOComm::openComm()
{
  // Check if FIFO exists already and make it if not
  struct stat buffer;
  if (stat(WRITE_FIFO_NAME, &buffer) != 0)
  {
    if (mkfifo(WRITE_FIFO_NAME, S_IRUSR | S_IWUSR) == -1)
      return false;
  }

  // Open Read Fifo
  // Keep trying to open till we get it to give time for other end to create it
  while ((rd_fifo = open(READ_FIFO_NAME, O_NONBLOCK | O_RDONLY)) < 0)
    sleep(1);

  // Open write fifo; it should exist at this time so don't retry if fail
  wr_fifo = open(WRITE_FIFO_NAME, O_WRONLY);
  return wr_fifo != -1;
}

bool FIFOComm::writeMsg(const char *name, double value)
{
  ostringstream ss;
  ss << value;
  return writeMsg(name, ss.str().c_str());
}

bool FIFOComm::writeMsg(const char *name, const char *value)
{
  ostringstream ss;
  ss << name << '=' << value << delim;

  const char *msg = ss.str().c_str(); // Full message to send
  size_t msgLen = strlen(msg);        // Length of message we are sending
  ssize_t nBytes;                     // Actual number of bytes written
  
  // Check if we can write, return if not since we don't want to block
/*  struct pollfd mpollfd = { .fd = fd, .events = POLLOUT };
  MOOSTrace("POLL: %i\n", poll(&mpollfd, 1, 0));
  if (!(poll(&mpollfd, 1, 0) > 0))
    return MOOSFail("Stream is not ready for writing");*/

  // Write to our FIFO
  nBytes = write(wr_fifo, msg, msgLen);
  if (nBytes < 0)
    return false; // "Error writing to frontseat"

  return true;
}

int FIFOComm::readMsg(char *buffer, int bufferLen)
{
 return read(rd_fifo, buffer, bufferLen);
}
