/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: CourseKeepMRAS.h                                */
/*    DATE: 2015-01-15                                      */
/************************************************************/

#ifndef CourseKeepMRAS_HEADER
#define CourseKeepMRAS_HEADER

class CourseKeepMRAS
{
public:
    CourseKeepMRAS();
    ~CourseKeepMRAS() {}

    double Run(double dfDesiredHeading, double dfMeasuredHeading, double dfMeasuredROT,
     double dfSpeed, double dfTime, bool bDoAdapt, bool bTurning);
    void SetParameters(double dfKStar, double dfTauStar, double dfZ, double dfWn,
        double dfBeta, double dfAlpha, double dfGamma, double dfXi,
        double dfRudderLimit, double dfCruisingSpeed, double dfShipLength,
        double dfMaxROT, bool bDecreaseAdapt, double dfRudderSpeed,
        double dfDeadband);
    static double TwoSidedLimit(double dfNumToLimit, double dfLimit);
    std::string GetStatusInfo();
    std::string GetDebugInfo();
    void GetDebugVariables(double * vars);
    void ResetModel(double dfHeading, double dfROT, double dfRudder);
    void SwitchController();
    double GetModelRudder();
    double GetTauStar();
    double GetTauM();
    double GetKStar();

private:
    void InitModel(double dfHeading, double dfROT, double dfSpeed);
    void UpdateModel(double dfMeasuredROT, double dfRudder, double dfSpeed,
        double dfDeltaT, bool bDoAdapt, bool bTurning);
    void UpdateRudderModel(double dfDeltaT);


private: // Configuration variables

    double m_dfZ;
    double m_dfWn;
    double m_dfBeta;
    double m_dfAlpha;
    double m_dfGamma;
    double m_dfXi;
    double m_dfRudderLimit;
    double m_dfMaxROT;
    double m_dfCruisingSpeed;
    double m_dfShipLength;
    double m_bDecreaseAdapt;
    double m_dfMaxROTInc;
    double m_dfDeadband;

 private: //Derived variables
    double m_dfTauM;
    double m_dfKm;

 private: // State variables
    double m_dfKp;
    double m_dfKd;
    double m_dfKi;

    double m_dfKmStar;
    double m_dfTaumStar;

    double m_dfPreviousTime;
    double m_dfPreviousHeading;
    double m_dfMeasuredHeading;
    double m_dfMeasuredROT;
    double m_dfInitTime;
    long   m_lIterations;
    bool   m_bFirstRun;
    bool   m_bControllerSwitch;
    bool   m_bParametersSet;
    double m_dfRudderOut;

    double m_dfKm0;
    double m_dfTaum0;
    double m_dfKim0;
    double m_dfKim;
    double m_dfMu;

    double m_dfModelHeading;
    double m_dfModelROT;
    double m_dfSeriesHeading;
    double m_dfSeriesROT;
    double m_dfPsiRefP;
    double m_dfPsiRefPP;
    double m_dfF;
    double m_dfModelRudder;
    double m_dfModelPhiDotDot;

    double m_dfRudderPos;
    double m_dfRudderSpeed;
};

#endif
