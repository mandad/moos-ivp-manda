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
    m_bControllerSwitch = false;
    m_bParametersSet = false;
    m_dfRudderOut = 0;
    //m_dfF = 1;
    m_dfMaxROTInc = 6;
    m_dfModelRudder = 0;

    //We should read this in as a parameter
    m_dfMu = 2;
}

void CourseKeepMRAS::SetParameters(double dfKStar, double dfTauStar, double dfZ,
    double dfBeta, double dfAlpha, double dfGamma, double dfXi,
    double dfRudderLimit, double dfCruisingSpeed, double dfShipLength,
    double dfMaxROT, bool bDecreaseAdapt, double dfRudderSpeed)
{
    m_dfKmStar = dfKStar;
    m_dfTaumStar = dfTauStar;
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

    m_lIterations = 0;
    m_bParametersSet = true;
}

double CourseKeepMRAS::Run(double dfDesiredHeading, double dfMeasuredHeading,
    double dfMeasuredROT, double dfSpeed, double dfTime)
{
    dfMeasuredHeading = angle180(dfMeasuredHeading);
    dfDesiredHeading = angle180(dfDesiredHeading);

    //m_dfRudderOut = 0;

    if (m_bFirstRun && m_bParametersSet) {
        //Otherwise this will nearly always be zero and result in incorrect
        //initial values for Kp, etc
        // NewHeading(m_dfCruisingSpeed);
        InitModel(dfMeasuredHeading, dfMeasuredROT, m_dfCruisingSpeed);
        m_dfInitTime = dfTime;
        MOOSTrace("Model and Controller Initialized.\n");

        m_bFirstRun = false;
    } else if (m_bControllerSwitch) {
        m_bControllerSwitch = false;
    } else {
        //Normal operation
        double dfDeltaT = dfTime - m_dfPreviousTime;
        //This includes both the series and parallel models
        UpdateRudderModel(dfDeltaT);
        UpdateModel(dfMeasuredROT, m_dfModelRudder, dfSpeed, dfDeltaT);

        // double dfTimeReduceFactor = 1;
        // if (m_bDecreaseAdapt) {
        //     dfTimeReduceFactor = m_dfXi / (1 + dfTime - m_dfCourseChangeTime);
        // }

        // Determine the PID constants
        // Kp does not change
        m_dfKd = (m_dfShipLength * 2 * sqrt(m_dfKp * m_dfKmStar * m_dfTaumStar) - 1) /
            (dfSpeed * m_dfKmStar);
        if (m_dfKd < 0)
            m_dfKd = 0;
        else if (m_dfKd > (m_dfKp * m_dfShipLength / dfSpeed))
            m_dfKd = m_dfKp * m_dfShipLength / dfSpeed;

        m_dfKi = m_dfKim;
        if (fabs(m_dfKi) > (10 * dfSpeed / m_dfCruisingSpeed)) {
            m_dfKi = TwoSidedLimit(m_dfKi, 10 * dfSpeed);
        }
    }
    MOOSTrace("PID Constants: Kp: %0.2f  Kd: %0.2f  Ki: %0.2f\n", m_dfKp, 
        m_dfKd, m_dfKi);
    //PID equation
    double heading_error = angle180(dfDesiredHeading - dfMeasuredHeading);
    m_dfRudderOut = m_dfKp * heading_error - m_dfKd * dfMeasuredROT + m_dfKi;

    m_dfPreviousTime = dfTime;
    m_dfPreviousHeading = dfDesiredHeading;
    m_dfMeasuredHeading = dfMeasuredHeading;

    //limit the rudder
    return TwoSidedLimit(m_dfRudderOut, m_dfRudderLimit);
}

void CourseKeepMRAS::InitModel(double dfHeading, double dfROT, double dfSpeed) {
    MOOSTrace("Mu: %0.2f\n", m_dfMu);
    m_dfKp = m_dfMu / 2;
    m_dfKd = (m_dfShipLength * 2 * sqrt(m_dfKp * m_dfKmStar * m_dfTaumStar) - 1) /
        (dfSpeed * m_dfKmStar);
    if (m_dfKd < m_dfKp) {
        m_dfKd = m_dfKp;
    } else if (m_dfKd > (m_dfKp * m_dfShipLength / dfSpeed)) {
        m_dfKd = m_dfKp * m_dfShipLength / dfSpeed;
    }
    m_dfKim = 0;

    //Model variables
    m_dfTauM = m_dfTaumStar * dfSpeed / m_dfShipLength;
    m_dfKm = m_dfKmStar * m_dfShipLength / dfSpeed;
    ResetModel(dfHeading, dfROT, dfSpeed);

    MOOSTrace("Adaptive Init: TauM*: %0.2f  Km*: %0.2f  Ki,m: %0.2f\n", m_dfTaumStar,
        m_dfKmStar, m_dfKim);
}

void CourseKeepMRAS::ResetModel(double dfHeading, double dfROT, double dfRudder) {
    m_dfModelHeading = dfHeading;
    m_dfModelROT = dfROT;
    m_dfModelPhiDotDot = m_dfKm / m_dfTauM * dfRudder - dfROT / m_dfTauM;
    MOOSTrace("Model Init: Y..: %0.2f  Y.: %0.2f  Y: %0.2f\n", m_dfModelPhiDotDot,
        m_dfModelROT, m_dfModelHeading);
    MOOSTrace("Model Init: TauM*: %0.2f  Km*: %0.2f  Ki,m: %0.2f\n", m_dfTaumStar,
        m_dfKmStar, m_dfKim);

    m_dfModelRudder = dfRudder;
    m_dfRudderOut = dfRudder;
}

void CourseKeepMRAS::SwitchController() {
    m_bControllerSwitch = true;
}

void CourseKeepMRAS::UpdateModel(double dfMeasuredROT, double dfRudder, 
    double dfSpeed, double dfDeltaT) {
    //Propagate model
    m_dfModelPhiDotDot = (m_dfKm * (dfRudder + m_dfKim) - m_dfModelROT) / m_dfTauM;
    m_dfModelROT += m_dfModelPhiDotDot * dfDeltaT;
    //this is the limit of ROT in this model
    if (fabs(m_dfModelROT) > fabs(m_dfKm * dfRudder)) {
        m_dfModelROT = m_dfKm * dfRudder;
    }
    m_dfModelHeading += m_dfModelROT * dfDeltaT;
    m_dfModelHeading = angle180(m_dfModelHeading);

    MOOSTrace("Process Vars: Y.: %0.2f  dT: %0.2f\n",dfMeasuredROT, dfDeltaT);
    MOOSTrace("Model Params: Km: %0.2f  Taum: %0.2f\n", m_dfKm, m_dfTauM);
    MOOSTrace("Model Update: Y..: %0.2f  Y.: %0.2f  Y: %0.2f  Rudder: %0.2f\n", 
        m_dfModelPhiDotDot, m_dfModelROT, m_dfModelHeading, dfRudder);

    // Process Vars: Y.: 0.00
    // Model Update: Y..: 0.00  Y.: 0.00  Y: 180.00  Rudder: 0.00
    // Adaptive Update: TauM*: inf  Km*: nan  Ki,m: 0.00
    // PID Constants: Kp: 1.00  Kd: nan  Ki: 0.00

    //Do adaptation
    double dfe = m_dfModelROT - dfMeasuredROT;
    double dfDeltaKmTm = (-m_dfBeta * dfe * (dfRudder - m_dfKim)) * dfDeltaT;
    double dfDeltaTmRecip = (m_dfAlpha * dfe * m_dfModelROT) * dfDeltaT;
    m_dfTaumStar = 1 / (1/m_dfTaumStar + dfDeltaTmRecip);
    if (m_dfTaumStar < 0.1) {
        m_dfTaumStar = 0.1;
    }
    m_dfKmStar += dfDeltaKmTm * m_dfTaumStar;
    if (m_dfKmStar < 0.1) {
        m_dfKmStar = 0.1;
    }
    m_dfKim -= m_dfGamma * dfe;

    MOOSTrace("Adaptive Update: TauM*: %0.2f  Km*: %0.2f  Ki,m: %0.2f\n", m_dfTaumStar,
        m_dfKmStar, m_dfKim);

    m_dfTauM = m_dfTaumStar * dfSpeed / m_dfShipLength;
    m_dfKm = m_dfKmStar * m_dfShipLength / dfSpeed;
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
    MOOSTrace("Model Rudder: %0.2f  Rudder Out: %0.2f\n", m_dfModelRudder, dfRudderOut);
}

double CourseKeepMRAS::GetModelRudder() {
    return m_dfModelRudder;
}

double CourseKeepMRAS::GetTauStar() {
    return m_dfTaumStar;
}

double CourseKeepMRAS::GetKStar() {
    return m_dfKmStar;
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
    vars[6] = m_dfTaumStar;  //m_dfSeriesHeading
    vars[7] = m_dfModelPhiDotDot; //m_dfSeriesROT
    vars[8] = m_dfTauM; //m_dfPsiRefP
    vars[9] = m_dfKm;   //m_dfPsiRefPP
    vars[10] = m_dfModelRudder;
}
