/************************************************************/
/*    NAME: Mike Bogochow                                   */
/*    ORGN: UNH                                             */
/*    FILE: UtilMOOSArduino.h                               */
/*    DATE: February 2014                                   */
/************************************************************/


#ifndef UTILMOOSARDUINO_H
#define UTILMOOSARDUINO_H

#include <list>
#include <string>

bool ValsFromString(std::list<std::string> &sVal, const std::string &sStr,
  const std::string &sTk, bool bInsensitive = true,
  const std::string delimiter = ",");

void strListToDblList(std::list<std::string> strList,
  std::list<double> &dblList);

#endif /* UTILMOOSARDUINO_H */
