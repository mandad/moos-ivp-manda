/************************************************************/
/*    NAME: Mike Bogochow                                   */
/*    ORGN: UNH                                             */
/*    FILE: UtilMOOSArduino.cpp                             */
/*    DATE: February 2014                                   */
/************************************************************/
/* A set of functions written for use in iMOOSArduino.      */
/************************************************************/

#include "MOOS/libMOOS/MOOSLib.h"
#include "UtilMOOSArduino.h"

// This is a rewrite of MOOSValFromString from MOOSUtilityFunctions.cpp.  It
// adds support for specifying the token for which key value pairs are
// separated.  Also now returns a list of strings through the out parameter
// to allow for detection of multiple copies of the key.
bool ValsFromString(std::list<std::string> &sVal,
  const std::string &sStr, const std::string &sTk, bool bInsensitive /*=true*/,
  const std::string delimiter /*=","*/)
{
  const char *whitespace = " \t";

	if(sTk.find(delimiter) != std::string::npos)
		return false;

  size_t nPos = std::string::npos;
  size_t k = 0;
  while((nPos = MOOSStrFind(sStr.substr(k), sTk, bInsensitive)) !=
    std::string::npos)
  {
    nPos += k;

    // We have the start of the token at nPos. We need to be careful here =
    // there could be many spaces between token and '='.
    size_t nEqualsPos = sStr.find('=', nPos);
    size_t nLastDelim = sStr.find_last_of(delimiter, nPos);
    size_t nLastChar;

    if(nLastDelim == std::string::npos)
    	nLastChar = sStr.find_first_not_of(whitespace, 0);

  	// Starting from previous delimiter, when is the first non-whitespace char?
    else
    	nLastChar = sStr.find_first_not_of(whitespace, nLastDelim + 1);

    if(nLastChar != nPos)
    { // Extra chars found
    	k = nPos + 1;
    	continue;
    }

  	// Look for a "="
    if(nEqualsPos != std::string::npos)
    { // There should only be whitespace twixt token and equals
      std::string t = sStr.substr(nPos + sTk.size(), nEqualsPos -
        (nPos + sTk.size()));
      MOOSTrimWhiteSpace(t);
      if(!t.empty())
      {
        //k = nEqualsPos;
        k = nPos + 1;
        continue;
      }

      //sVal.clear();
      //sVal = "";

      int nDelimPos = sStr.find(delimiter, nEqualsPos);

      std::string sub = sStr.substr(nEqualsPos + 1, nDelimPos - nEqualsPos - 1);
      MOOSTrimWhiteSpace(sub);

      sVal.push_front(sub);
      //sVal.append(sStr, nEqualsPos + 1, nDelimPos - nEqualsPos - 1);

      k = nPos + 1;
      continue;
      //return true;
    }

    else
      return false;
  } /* while */

  if (sVal.empty())
    return false;

  else
    return true;
} /* ValsFromString */

/**
 * Convert a list of strings to a list of doubles.  Any values within the
 * string list which are non-numeric will not be added to the double list.
 */
void strListToDblList(std::list<std::string> strList,
  std::list<double> &dblList)
{
  for (std::list<std::string>::iterator it = strList.begin();
    it != strList.end(); it++)
  {
    if (MOOSIsNumeric(*it))
      dblList.push_back(MOOS::StringToDouble(*it));
  }
} /* strListToDblList */
