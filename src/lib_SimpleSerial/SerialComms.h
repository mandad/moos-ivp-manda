/*
 * SerialComms.h
 *
 *  Created on:   July 20, 2014
 *      Author:   Alon Yaari
 *      BASED ON:
 *          arduino-serial-lib -- simple library for reading/writing serial ports
 *          2006-2013, Tod E. Kurt, http://todbot.com/blog/
 *
 */

#ifndef SERIALCOMMS_H_
#define SERIALCOMMS_H_

#include <stdint.h>   // Standard types
#include <deque>
#include "MOOS/libMOOS/MOOSLib.h"

#define BUF_SIZE 1024
#define TIMEOUT  30000      // 30 seconds (in ms)

class SerialComms {

public:
                SerialComms(std::string port, int baud, std::string& errMsg);
    virtual     ~SerialComms() {};
    bool        Run();
    static bool dispatch(void* param);

    void        Set_Delims(char beg, char end, std::string ignore);
    bool        IsListening() { return bGoodSerialComms; };
    std::string GetNextSentence();
    bool        WriteToSerialPort(std::string str);
    bool        SerialLoop();
    bool        IsGoodSerialComms() { return bGoodSerialComms; };
    int         DataAvailable();
    int         serialport_flush();

private:
    bool        serialport_init(const char* serialport, int baud, std::string& errMsg);
    int         serialport_close();
    int         serialport_writebyte(uint8_t b, std::string& errMsg);
    int         serialport_write(const char* str, std::string& errMsg);
    bool        SerialIncoming(std::string& errMsg);
    void        AddToProcessingQueue(std::string& str);
    std::string GetLastErrorMsg();

    CMOOSThread serialThread;

    char        m_char_beg;
    char        m_char_end;
    std::string m_ignore;
    bool        bGoodSerialComms;
    int         fd;
    bool        bQuit;
    std::string lastErrorMsg;
    std::deque<std::string>   inLines;
    char        buf[BUF_SIZE];
    std::string strBuf;
};

#endif











//
