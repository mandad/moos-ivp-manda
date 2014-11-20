/************************************************************/
/*    NAME: Mike Bogochow                                   */
/*    ORGN: UNH                                             */
/*    FILE: IMOOSComm.h                                     */
/*    DATE: Mar, 2014                                       */
/************************************************************/
/* This is an interface for handling communications with    */
/* processes outside of MOOS.  Classes implementing this    */
/* interface should be used by a payload autonomy interface */
/* for forwarding MOOS variables to the frontseat.          */
/************************************************************/

#ifndef IMOOSCOMM_H
#define IMOOSCOMM_H

class IMOOSComm
{
public:
  virtual ~IMOOSComm() {};

  /**
   * Open all communication channels necessary.
   *
   */
  virtual bool openComm()=0;

  /**
   * Write the given key value pair to the frontseat.
   *
   * @return true  if the value is successfully written to the frontseat,
   *         false otherwise
   */
  virtual bool writeMsg(const char *name, double value)=0;

  /**
   * Write the given key value pair to the frontseat.
   *
   * @return true  if the value is successfully written to the frontseat,
   *         false otherwise
   */
  virtual bool writeMsg(const char *name, const char *value)=0;

  /**
   * Read a message from the frontseat.
   */
  virtual int readMsg(char *buffer, int bufferLen)=0;
};

#endif /* IMOOSCOMM_H */

