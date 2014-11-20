
#ifndef COMMTYPE_H
#define COMMTYPE_H

#include <string>

enum CommType { UNKNOWN, FIFO, ARDUINO, NETSERVER, NETCLIENT };

CommType getCommType(std::string commType);
std::string getStringFromCommType(CommType commType);

#endif /* COMMTYPE_H */
