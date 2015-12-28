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

#define USE_SERIES_MODEL true

using namespace std;

CourseChangeMRAS::CourseChangeMRAS() {
    m_bFirstRun = true;
    m_bParametersSet = false;
    m_dfRudderOut = 0;
    m_dfF = 1;
}

CourseChangeMRAS::CourseChangeMRAS(double dfKStar, double dfTauStar, double dfZ, 
    double dfBeta, double dfAlpha, double dfGamma, double dfXi, 
    double dfRudderLimit, double dfCruisingSpeed, double dfShipLength, double dfMaxROT) 
{
    m_dfKStar = dfKStar;
    m_dfTauStar = dfTauStar;
    m_dfZ = dfZ;
    m_dfBeta = dfBeta;
    m_dfAlpha = dfAlpha;
    m_dfGamma = dfGamma;
    m_dfXi = dfXi;
    m_dfRudderLimit = dfRudderLimit;
    m_dfMaxROT = dfMaxROT;
    m_dfCruisingSpeed = dfCruisingSpeed;
    m_dfShipLength = dfShipLength;

    m_dfTauM = 0.5 * m_dfTauStar * m_dfShipLength / m_dfCruisingSpeed;
    m_dfKpm = 1 / (4 * m_dfZ * m_dfZ * m_dfTauM);
    m_dfP12 = m_dfTauM / m_dfKpm;
    m_dfP22 = m_dfTauM * m_dfTauM / m_dfKpm + m_dfTauM;
    m_dfRudderOut = 0;

    m_lIterations = 0;
    m_bFirstRun = true;
    m_bParametersSet = true;
}

void CourseChangeMRAS::SetParameters(double dfKStar, double dfTauStar, double dfZ, 
    double dfBeta, double dfAlpha, double dfGamma, double dfXi, 
    double dfRudderLimit, double dfCruisingSpeed, double dfShipLength, double dfMaxROT)
{
    m_dfKStar = dfKStar;
    m_dfTauStar = dfTauStar;
    m_dfZ = dfZ;
    m_dfBeta = dfBeta;
    m_dfAlpha = dfAlpha;
    m_dfGamma = dfGamma;
    m_dfXi = dfXi;
    m_dfRudderLimit = dfRudderLimit;
    m_dfMaxROT = dfMaxROT;
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
    dfMeasuredHeading = angle180(dfMeasuredHeading);
    dfDesiredHeading = angle180(dfDesiredHeading);

    m_dfRudderOut = 0;

    if (m_bFirstRun || (abs(angle180(dfDesiredHeading - m_dfPreviousHeading)) > 5)) {
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
        
        double dfe = m_dfModelHeading - dfMeasuredHeading;
        dfe = angle180(dfe);
        double dfeDot = m_dfModelROT - dfMeasuredROT;

        //Adapt the parameters
        double dfErrorFactor = m_dfP12 * dfe + m_dfP22 * dfeDot;
        double dfTimeReduceFactor = m_dfXi / (1 + dfTime - m_dfCourseChangeTime);
        //If using series model, desired heading should be Phi''r
        m_dfKp += m_dfBeta * dfTimeReduceFactor * dfErrorFactor *
            (angle180(m_dfPsiRefPP - dfMeasuredHeading)) * dfDeltaT;
        if (m_dfKp < 0)
            m_dfKp = 0;
        m_dfKd += m_dfAlpha * dfTimeReduceFactor * dfErrorFactor * dfMeasuredROT 
            * dfDeltaT;
        if (m_dfKd < 0)
            m_dfKd = 0;
        m_dfKi += m_dfGamma * dfErrorFactor * dfDeltaT;
        MOOSTrace("Updated constants\n");


    }
    //PID equation
    double heading_error = angle180(m_dfPsiRefP - dfMeasuredHeading);
    m_dfRudderOut = m_dfKp * heading_error - m_dfKd * dfMeasuredROT + m_dfKi;
    //limit the rudder
    //m_dfRudderOut = 

    m_dfPreviousTime = dfTime;
    m_dfPreviousHeading = dfDesiredHeading;
    m_dfMeasuredHeading = dfMeasuredHeading;

    return TwoSidedLimit(m_dfRudderOut, m_dfRudderLimit);
}

bool CourseChangeMRAS::NewHeading(double dfSpeed) {
    // Try to avoid problems with really small speeds since it is in the 
    // denominator of the constants
    if (dfSpeed < 0.1) {
        dfSpeed = 0.1;
    }

    //from literature:  
    //m_dfKp0 = 2.5 * m_dfCruisingSpeed / dfSpeed;
    m_dfKp0 = 1 * m_dfCruisingSpeed / dfSpeed;
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
    //Series Model
    m_dfSeriesHeading = dfHeading;
    m_dfSeriesROT = dfROT;
    m_dfPsiRefPP = dfHeading;
    m_dfPsiRefP = dfHeading;

    //Parallel Model
    m_dfModelHeading = dfHeading;
    m_dfModelROT = dfROT;
}

void CourseChangeMRAS::UpdateModel(double dfDesiredHeading, double dfDeltaT) {
    //Update series model
    //This model serves to include nonlinearities such as saturation of rudder
    //and rate of turn from mechanical or user set limits

    // #if USE_SERIES_MODEL
    m_dfSeriesHeading += m_dfSeriesROT * dfDeltaT;
    m_dfSeriesHeading = angle180(m_dfSeriesHeading);

    m_dfF = 1;
    if (fabs(m_dfRudderOut) > m_dfRudderLimit) {
        m_dfF = m_dfRudderLimit / fabs(m_dfRudderOut);
    }
    //We split these variables out because they are used to calculate inputs to
    //the parallel model and PID controller
    double dfROTModDiff = TwoSidedLimit(angle180(dfDesiredHeading - m_dfSeriesHeading) 
        * m_dfKpm, m_dfMaxROT);
    double dfFModDiff = m_dfF * dfROTModDiff;
    m_dfSeriesROT += (dfFModDiff / m_dfTauM  
        - 1/m_dfTauM * m_dfSeriesROT) * dfDeltaT;
    //Input to PID as desired heading
    m_dfPsiRefP = angle180(dfROTModDiff * 1/m_dfKpm + m_dfSeriesHeading);
    //Input to parallel model as desired heading
    m_dfPsiRefPP = angle180(dfFModDiff * 1/m_dfKpm + m_dfSeriesHeading);
    // #else
    // m_dfPsiRefPP = dfDesiredHeading;
    // m_dfPsiRefP = dfDesiredHeading;
    // #endif

    //Update Parallel Model
    //uses output of series model as desired heading
    m_dfModelHeading += m_dfModelROT * dfDeltaT;
    m_dfModelHeading = angle180(m_dfModelHeading);

    //need to reference the PsiRefPP - dfModelHeading to angle180
    m_dfModelROT += ((m_dfKpm / m_dfTauM * (angle180(m_dfPsiRefPP - m_dfModelHeading)))
        - 1/m_dfTauM * m_dfModelROT) * dfDeltaT;
}

double CourseChangeMRAS::TwoSidedLimit(double dfNumToLimit, double dfLimit) {
    if (fabs(dfNumToLimit) > dfLimit) {
        return copysign(dfLimit, dfNumToLimit);
    }
    return dfNumToLimit;
}

string CourseChangeMRAS::GetStatusInfo() {
    stringstream info;
    info << m_dfKp << "|" << m_dfKd << "|" << m_dfKi << "|" << m_dfModelHeading
        << "|" << m_dfMeasuredHeading << "|" << m_dfPreviousHeading;
    return info.str();
}

string CourseChangeMRAS::GetDebugInfo() {
    stringstream info;
    info << m_dfPsiRefPP << "|" << m_dfPsiRefP << "|" << m_dfRudderOut << "|" 
    << m_dfSeriesHeading << "|" << m_dfModelROT << "|" << m_dfSeriesROT;
    return info.str();
}

void CourseChangeMRAS::GetDebugVariables(double * vars) {
    vars[0] = m_dfKp;
    vars[1] = m_dfKd;
    vars[2] = m_dfKi;
    vars[3] = m_dfRudderOut;
    vars[4] = m_dfModelHeading;
    vars[5] = m_dfModelROT;
    vars[6] = m_dfSeriesHeading;
    vars[7] = m_dfSeriesROT;
    vars[8] = m_dfPsiRefP;
    vars[9] = m_dfPsiRefPP;
    vars[10] = m_dfF;
}

