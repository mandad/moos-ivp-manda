/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: CourseKeepMRAS.cpp                              */
/*    DATE: 2015-01-15                                      */
/************************************************************/

#include "MOOS/libMOOS/MOOSLib.h"
#include "AngleUtils.h"
#include "CourseKeepMRAS.h"
#include <math.h>

#define USE_SERIES_MODEL true
#define LIMIT_ROT_INC false
#define RESET_THRESHOLD 5
#define KP_LIMIT 2.5

using namespace std;

CourseKeepMRAS::CourseKeepMRAS() {
    m_bFirstRun = true;
    m_bParametersSet = false;
    m_dfRudderOut = 0;
    m_dfF = 1;
    m_dfMaxROTInc = 6;
    m_dfModelRudder = 0;
}

void CourseKeepMRAS::SetParameters(double dfKStar, double dfTauStar, double dfZ,
    double dfBeta, double dfAlpha, double dfGamma, double dfXi,
    double dfRudderLimit, double dfCruisingSpeed, double dfShipLength,
    double dfMaxROT, bool bDecreaseAdapt, double dfRudderSpeed)
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
    m_bDecreaseAdapt = bDecreaseAdapt;
    m_dfRudderSpeed = dfRudderSpeed;

    m_dfTauM = 0.5 * m_dfTauStar * m_dfShipLength / m_dfCruisingSpeed;
    m_dfKpm = 1 / (4 * m_dfZ * m_dfZ * m_dfTauM);
    m_dfP12 = m_dfTauM / m_dfKpm;
    m_dfP22 = m_dfTauM * m_dfTauM / m_dfKpm + m_dfTauM;

    m_lIterations = 0;
    m_bParametersSet = true;
}

double CourseKeepMRAS::Run(double dfDesiredHeading, double dfMeasuredHeading,
    double dfMeasuredROT, double dfSpeed, double dfTime)
{
    dfMeasuredHeading = angle180(dfMeasuredHeading);
    dfDesiredHeading = angle180(dfDesiredHeading);

    //m_dfRudderOut = 0;

    if (m_bFirstRun || (abs(angle180(dfDesiredHeading - m_dfPreviousHeading))
        > RESET_THRESHOLD)) {
        //Initial with no adaptation
        if (m_bFirstRun) {
            //Otherwise this will nearly always be zero and result in incorrect
            //initial values for Kp, etc
            NewHeading(m_dfCruisingSpeed);
        } else {
            NewHeading(dfSpeed);
        }
        ResetModel(dfMeasuredHeading, dfMeasuredROT);
        m_dfCourseChangeTime = dfTime;
        MOOSTrace("Model and Controller Initialized.\n");

        m_bFirstRun = false;
    } else {
        //Normal operation
        double dfDeltaT = dfTime - m_dfPreviousTime;
        //This includes both the series and parallel models
        UpdateRudderModel(dfDeltaT);
        UpdateModel(dfDesiredHeading, dfDeltaT);

        double dfe = m_dfModelHeading - dfMeasuredHeading;
        dfe = angle180(dfe);
        double dfeDot = m_dfModelROT - dfMeasuredROT;

        //Adapt the parameters
        double dfErrorFactor = m_dfP12 * dfe + m_dfP22 * dfeDot;
        double dfTimeReduceFactor = 1;
        if (m_bDecreaseAdapt) {
            dfTimeReduceFactor = m_dfXi / (1 + dfTime - m_dfCourseChangeTime);
        }
        //If using series model, desired heading should be Phi''r
        m_dfKp += m_dfBeta * dfTimeReduceFactor * dfErrorFactor *
            (angle180(m_dfPsiRefPP - dfMeasuredHeading)) * dfDeltaT;
        if (m_dfKp < 0)
            m_dfKp = 0;
        else if (m_dfKp > KP_LIMIT)
             m_dfKp = KP_LIMIT;

        m_dfKd -= m_dfAlpha * dfTimeReduceFactor * dfErrorFactor * dfMeasuredROT
            * dfDeltaT;
        if (m_dfKd < 0)
            m_dfKd = 0;
        else if (m_dfKd0 > (m_dfKp * m_dfShipLength / dfSpeed))
            m_dfKd = m_dfKp * m_dfShipLength / dfSpeed;

        m_dfKi += m_dfGamma * dfErrorFactor * dfDeltaT;
        //MOOSTrace("Updated constants\n");
    }
    //PID equation
    double heading_error = angle180(m_dfPsiRefP - dfMeasuredHeading);
    m_dfRudderOut = m_dfKp * heading_error - m_dfKd * dfMeasuredROT + m_dfKi;

    m_dfPreviousTime = dfTime;
    m_dfPreviousHeading = dfDesiredHeading;
    m_dfMeasuredHeading = dfMeasuredHeading;

    //limit the rudder
    return TwoSidedLimit(m_dfRudderOut, m_dfRudderLimit);
}

bool CourseKeepMRAS::NewHeading(double dfSpeed) {
    // Try to avoid problems with really small speeds since it is in the
    // denominator of the constants
    if (dfSpeed < 0.1) {
        //If we aren't resetting, seed it higher - already done in calling code
        // if (RESET_THRESHOLD >= 180) {
        //     dfSpeed = m_dfCruisingSpeed;
        // } else {
            dfSpeed = 0.1;
        // }
    }

    //from literature:
    //m_dfKp0 = 2.5 * m_dfCruisingSpeed / dfSpeed;
    m_dfKp0 = 1.0 * m_dfCruisingSpeed / dfSpeed;
    if (m_dfKp0 > KP_LIMIT) {
        m_dfKp0 = KP_LIMIT;
    }

    //This equation is incorrect in Van Amerongen
    m_dfKd0 = (m_dfShipLength * 2 * m_dfZ * sqrt(m_dfKp0 * m_dfKStar * m_dfTauStar) - 1) /
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

void CourseKeepMRAS::ResetModel(double dfHeading, double dfROT) {
    //Series Model
    m_dfSeriesHeading = dfHeading;
    m_dfSeriesROT = dfROT;
    m_dfPsiRefPP = dfHeading;
    m_dfPsiRefP = dfHeading;
    m_dfx2 = 0;

    //Parallel Model
    m_dfModelHeading = dfHeading;
    m_dfModelROT = dfROT;
}

void CourseKeepMRAS::UpdateModel(double dfDesiredHeading, double dfDeltaT) {
 
}

void CourseKeepMRAS::UpdateRudderModel(double dfDeltaT) {
    // Update a model of the actual rudder location
    // This is needed for simple systems which do not have a sensor to provide this
    //feedback
    double dfRudderOut = TwoSidedLimit(m_dfRudderOut, m_dfRudderLimit);
    double dfRudderInc = dfDeltaT * m_dfRudderSpeed;
    double dfRudderDiff = dfRudderOut - m_dfModelRudder;
    if (fabs(dfRudderDiff) > dfRudderInc) {
        m_dfModelRudder += copysign(dfRudderInc, dfRudderDiff);
    } else {
        m_dfModelRudder = dfRudderOut;
    }
    MOOSTrace("Model Rudder: %0.2f\n", m_dfModelRudder);
}

double CourseKeepMRAS::TwoSidedLimit(double dfNumToLimit, double dfLimit) {
    if (fabs(dfNumToLimit) > dfLimit) {
        return copysign(dfLimit, dfNumToLimit);
    }
    return dfNumToLimit;
}

string CourseKeepMRAS::GetStatusInfo() {
    stringstream info;
    info << m_dfKp << "|" << m_dfKd << "|" << m_dfKi << "|" << m_dfModelHeading
        << "|" << m_dfMeasuredHeading << "|" << m_dfPreviousHeading;
    return info.str();
}

string CourseKeepMRAS::GetDebugInfo() {
    stringstream info;
    info << m_dfPsiRefPP << "|" << m_dfPsiRefP << "|" << m_dfRudderOut << "|"
    << m_dfSeriesHeading << "|" << m_dfModelROT << "|" << m_dfSeriesROT;
    return info.str();
}

void CourseKeepMRAS::GetDebugVariables(double * vars) {
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
    vars[10] = m_dfModelRudder;
}
