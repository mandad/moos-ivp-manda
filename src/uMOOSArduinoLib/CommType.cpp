/************************************************************/
/*    NAME: Mike Bogochow, Jeff Masucci, Cody Noel          */
/*    ORGN: UNH                                             */
/*    FILE: CommType.cpp                                    */
/*    DATE: March 2014                                      */
/************************************************************/

#include "CommType.h"

#include <string>

/**
 * Get the CommType value for the given string value.
 *
 * @param commType the string value whose CommType value should be returned
 */
CommType getCommType(std::string commType)
{
  if (commType == "FIFO")
    return FIFO;
  else if (commType == "ARDUINO")
    return ARDUINO;
  else
    return UNKNOWN;
}

/**
 * Get the string value for the given CommType value.
 *
 * @param commType the enum value whose string value should be returned
 */
std::string getStringFromCommType(CommType commType)
{
  switch(commType)
  {
  case UNKNOWN:
    return "UNKNOWN";
  case FIFO:
    return "FIFO";
  case ARDUINO:
    return "ARDUINO";
  } 
}
