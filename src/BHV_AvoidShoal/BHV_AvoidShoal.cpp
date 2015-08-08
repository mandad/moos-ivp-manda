/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_AvoidShoal.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_AvoidShoal.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_AvoidShoal::BHV_AvoidShoal(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_AvoidShoal::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "foo") && isNumber(val)) {
    // Set local member variables here
    return(true);
  }
  else if (param == "bar") {
    // return(setBooleanOnString(m_my_bool, val));
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_AvoidShoal::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_AvoidShoal::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_AvoidShoal::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_AvoidShoal::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_AvoidShoal::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_AvoidShoal::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_AvoidShoal::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_AvoidShoal::onRunState()
{
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;



  // Part N: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}

