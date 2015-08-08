/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_AvoidShoal.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef AvoidShoal_HEADER
#define AvoidShoal_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_AvoidShoal : public IvPBehavior {
public:
  BHV_AvoidShoal(IvPDomain);
  ~BHV_AvoidShoal() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();

protected: // Local Utility functions

protected: // Configuration parameters

protected: // State variables
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_AvoidShoal(domain);}
}
#endif
