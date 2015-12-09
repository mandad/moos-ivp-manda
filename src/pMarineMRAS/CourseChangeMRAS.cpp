/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: CourseChangeMRAS.cpp                            */
/*    DATE: 2015-12-06                                      */
/************************************************************/

#include "MOOS/libMOOS/MOOSLib.h"
#include "AngleUtils.h"
#include "CourseChangeMRAS.h"
#include <math.h>

using namespace std;

CourseChangeMRAS::CourseChangeMRAS() {
    m_bFirstRun = true;
    m_bParametersSet = false;

}

CourseChangeMRAS::CourseChangeMRAS(double dfKStar, double dfTauStar, double dfZ, 
    double dfBeta, double dfAlpha, double dfGamma, double dfXi, 
    double dfRudderLimit, double dfCruisingSpeed, double dfShipLength) 
{
    m_dfKStar = dfKStar;
    m_dfTauStar = dfTauStar;
    m_dfZ = dfZ;
    m_dfBeta = dfBeta;
    m_dfAlpha = dfAlpha;
    m_dfGamma = dfGamma;
    m_dfXi = dfXi;
    m_dfRudderLimit = dfRudderLimit;
    m_dfCruisingSpeed = dfCruisingSpeed;
    m_dfShipLength = dfShipLength;

    m_dfTauM = 0.5 * m_dfTauStar * m_dfShipLength / m_dfCruisingSpeed;
    m_dfKpm = 1 / (4 * m_dfZ * m_dfZ * m_dfTauM);
    m_dfP12 = m_dfTauM / m_dfKpm;
    m_dfP22 = m_dfTauM * m_dfTauM / m_dfKpm + m_dfTauM;

    m_lIterations = 0;
    m_bFirstRun = true;
    m_bParametersSet = true;
}

void CourseChangeMRAS::SetParameters(double dfKStar, double dfTauStar, double dfZ, 
    double dfBeta, double dfAlpha, double dfGamma, double dfXi, 
    double dfRudderLimit, double dfCruisingSpeed, double dfShipLength)
{
    m_dfKStar = dfKStar;
    m_dfTauStar = dfTauStar;
    m_dfZ = dfZ;
    m_dfBeta = dfBeta;
    m_dfAlpha = dfAlpha;
    m_dfGamma = dfGamma;
    m_dfXi = dfXi;
    m_dfRudderLimit = dfRudderLimit;
    m_dfCruisingSpeed = dfCruisingSpeed;
    m_dfShipLength = dfShipLength;

    m_dfTauM = 0.5 * m_dfTauStar * m_dfShipLength / m_dfCruisingSpeed;
    m_dfKpm = 1 / (4 * m_dfZ * m_dfZ * m_dfTauM);
    m_dfP12 = m_dfTauM / m_dfKpm;
    m_dfP22 = m_dfTauM * m_dfTauM / m_dfKpm + m_dfTauM;

    m_lIterations = 0;
    m_bParametersSet = true;
}

double CourseChangeMRAS::Run(double dfDesiredHeading, double dfMeasuredHeading, 
    double dfMeasuredROT, double dfSpeed, double dfTime)
{
    double dfRudderOut = 0;

    if (m_bFirstRun || (abs(dfDesiredHeading - m_dfPreviousHeading) > 5)) {
        //Initial with no adaptation
        NewHeading(dfSpeed);
        ResetModel(dfMeasuredHeading, dfMeasuredROT);
        m_dfCourseChangeTime = dfTime;
        MOOSTrace("Model and Controller Initialized.\n");

        m_bFirstRun = false;
    } else {
        //Normal operation
        double dfDeltaT = dfTime - m_dfPreviousTime;
        //DesiredHeading would be phi''r if using series limit model
        UpdateModel(dfDesiredHeading, dfDeltaT);

        dfMeasuredHeading = angle180(dfMeasuredHeading);
        double dfe = m_dfModelHeading - dfMeasuredHeading;
        dfe = angle180(dfe);
        double dfeDot = m_dfModelROT - dfMeasuredROT;

        //Adapt the parameters
        double dfErrorFactor = m_dfP12 * dfe + m_dfP22 * dfeDot;
        double dfTimeReduceFactor = m_dfXi / (1 + dfTime - m_dfCourseChangeTime);
        //If using series model, desired heading should be Phi''r
        m_dfKp += m_dfBeta * dfTimeReduceFactor * dfErrorFactor *
            (dfDesiredHeading - dfMeasuredHeading) * dfDeltaT;
        m_dfKd += m_dfAlpha * dfTimeReduceFactor* dfErrorFactor * dfMeasuredROT 
            * dfDeltaT;
        m_dfKi += m_dfGamma * dfTimeReduceFactor* dfErrorFactor * dfDeltaT;
        MOOSTrace("Updated constants\n");


    }
    //PID equation
    dfRudderOut = m_dfKp * (dfDesiredHeading - dfMeasuredHeading) 
        - m_dfKd * dfMeasuredROT + m_dfKi;
    //limit the rudder
    if(fabs(dfRudderOut) >= m_dfRudderLimit) {        
        dfRudderOut = copysign(m_dfRudderLimit, dfRudderOut);
    }

    m_dfPreviousTime = dfTime;
    m_dfPreviousHeading = dfDesiredHeading;

    return dfRudderOut;
}

bool CourseChangeMRAS::NewHeading(double dfSpeed) {
    // Try to avoid problems with really small speeds since it is in the 
    // denominator of the constants
    if (dfSpeed < 0.1) {
        dfSpeed = 0.1;
    }

    m_dfKp0 = 2.5 * m_dfCruisingSpeed / dfSpeed;
    if (m_dfKp0 > 5) {
        m_dfKp0 = 5;
    }

    m_dfKd0 = (m_dfShipLength * 2 * m_dfZ * sqrt(m_dfKp0 * m_dfKStar * m_dfTauStar - 1)) / 
        (dfSpeed * m_dfKStar);
    if (m_dfKd0 < m_dfKp0) {
        m_dfKd0 = m_dfKp0;
    } else if (m_dfKd0 > (m_dfKp0 * m_dfShipLength / dfSpeed)) {
        m_dfKd0 = m_dfKp0 * m_dfShipLength / dfSpeed;
    }

    if (m_bFirstRun) {
        m_dfKi0 = 0;
    } else {
        //should be average of last estimation from course keeping
        //for now just reset it anyway (or maybe leave same)
        m_dfKi0 = 0;
    }

    //Don't think it actually needs both variables
    m_dfKp = m_dfKp0;
    m_dfKd = m_dfKd0;
    m_dfKi = m_dfKi0;

    return true;
}

void CourseChangeMRAS::ResetModel(double dfHeading, double dfROT) {
    m_dfModelHeading = dfHeading;
    m_dfModelROT = dfROT;
}

void CourseChangeMRAS::UpdateModel(double dfDesiredHeading, double dfDeltaT) {
    m_dfModelHeading += m_dfModelROT * dfDeltaT;
    m_dfModelROT += ((m_dfKpm / m_dfTauM * (dfDesiredHeading - m_dfModelHeading))
        - 1/m_dfTauM * m_dfModelROT) * dfDeltaT;

}

string CourseChangeMRAS::GetStatusInfo() {
    stringstream info;
    info << m_dfKp << "|" << m_dfKd << "|" << m_dfKi << "|" << m_dfModelHeading;
    return info.str();
}


