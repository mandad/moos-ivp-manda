/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: UNH, Durham NH                                            */
/*    FILE: GP9.cpp                                        */
/*    DATE: Apr 15th 2015                                                */
/************************************************************/

#include <iterator>
#include <string>
#include "MBUtils.h"
#include "ACTable.h"
#include "GP9.h"

using namespace std;

const char VERSION[10] = "0.0.1";   // gp9_driver version

// Don't try to be too clever. Arrival of this message triggers
// us to publish everything we have.
const uint8_t TRIGGER_PACKET = DREG_EULER_PHI_THETA;

//---------------------------------------------------------
// Constructor

GP9::GP9()
  : defaultBaudRate(115200),
  sensor(NULL)
{
  baudRate = defaultBaudRate;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool GP9::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     //if(key == "FOO") 
     //  cout << "great!";

     if(key != "APPCAST_REQ") // handle by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool GP9::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GP9::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!

  // triggered by arrival of last message packet
  if (ser.isOpen())
  {
    if (sensor.receive(&registers) == TRIGGER_PACKET)
    {
      publishMsgs(registers);
    }
  }
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GP9::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  // STRING_LIST::iterator p;
  // for(p=sParams.begin(); p!=sParams.end(); p++) {
  //   string orig  = *p;
  //   string line  = *p;
  //   string param = toupper(biteStringX(line, '='));
  //   string value = line;

  //   bool handled = false;
  //   if(param == "FOO") {
  //     handled = true;
  //   }
  //   else if(param == "BAR") {
  //     handled = true;
  //   }

  //   if(!handled)
  //     reportUnhandledConfigWarning(orig);

  // }

  // Set Up Serial Port
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  if (!m_MissionReader.GetConfigurationParam("SerialPort", serialPort))
    {
      MOOSTrace("Warning: parameter 'SerialPort' not specified.\n");
      MOOSTrace("Terminating\n");
      exit(-1);
    }

  if (!m_MissionReader.GetConfigurationParam("BaudRate", baudRate))
  {
    MOOSTrace("Warning: parameter 'BaudRate' not specified.  Using "
      "default baud rate (%i)\n", defaultBaudRate);
  }
  

  ser.setPort(serialPort);
  ser.setBaudrate(baudRate);
  serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
  ser.setTimeout(to);

  // Initialize covariance. The GP9 sensor does not provide covariance values so,
  //   by default, this driver provides a covariance array of all zeros indicating
  //   "covariance unknown" as advised in sensor_msgs/Imu.h.
  // This param allows the user to specify alternate covariance values if needed.

  std::string covariance;
  char cov[200];
  char *ptr1;

  if (!m_MissionReader.GetConfigurationParam("Covariance", covariance))
  {
    MOOSTrace("Using Default Covariance");
    covariance = "0 0 0 0 0 0 0 0 0";
  }
  snprintf(cov, sizeof(cov), "%s", covariance.c_str());

  char* p = strtok_r(cov, " ", &ptr1);           // point to first value
  for (int iter = 0; iter < 9; iter++)
  {
    if (p) covar[iter] = atof(p);                // covar[] is global var
    else  covar[iter] = 0.0;
    p = strtok_r(NULL, " ", &ptr1);              // point to next value (nil if none)
  }

  // Real Time Loop
  first_failure = true;
  try
  {
    ser.open();
  }
  catch(const serial::IOException& e)
  {
      MOOSFail("gp9_driver ver %s unable to connect to port.", VERSION);
  }
  if (ser.isOpen())
  {
    MOOSTrace("gp9_driver ver %s connected to serial port.", VERSION);
    first_failure = true;
    try
    {
      //Configure the comms rates/options
      sensor = gp9::Comms(&ser);
      configureSensor(&sensor);
    }
    catch(const std::exception& e)
    {
      if (ser.isOpen()) ser.close();
      MOOSTrace(e.what());
      MOOSTrace("Attempting reconnection after error.");
    }
  }
  else
  {
    if (first_failure)
    {
    MOOSTrace("Could not connect to serial device "
        + serialPort + ". Trying again on next iteration");
    }
    first_failure = false;
  }

  registerVariables();

  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void GP9::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool GP9::buildReport() 
{
  m_msgs << "============================================ \n";
  m_msgs << "GP9 Driver:                                  \n";
  m_msgs << "============================================ \n";

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}

//=============================================================
// GP9 Specific Functions
//=============================================================


/**
 * Function generalizes the process of writing an XYZ vector into consecutive
 * fields in GP9 registers.
 */
// template<typename RegT>
// void GP9::configureVector3(gp9::Comms* sensor, const gp9::Accessor<RegT>& reg,
//     std::string param, std::string human_name)
// {
//   if (reg.length != 3)
//   {
//     throw std::logic_error("configureVector3 may only be used with 3-field registers!");
//   }

//   if (ros::param::has(param))
//   {
//     double x, y, z;
//     ros::param::get(param + "/x", x);
//     ros::param::get(param + "/y", y);
//     ros::param::get(param + "/z", z);
//     ROS_INFO_STREAM("Configuring " << human_name << " to ("
//                     << x << ", " << y << ", " << z << ")");
//     reg.set_scaled(0, x);
//     reg.set_scaled(1, y);
//     reg.set_scaled(2, z);
//     if (sensor->sendWaitAck(reg))
//     {
//       throw std::runtime_error("Unable to configure vector.");
//     }
//   }
// }

/**
 * Function generalizes the process of commanding the GP9 via one of its command
 * registers.
 */
template<typename RegT>
void GP9::sendCommand(gp9::Comms* sensor, const gp9::Accessor<RegT>& reg, std::string human_name)
{
  MOOSTrace("Sending command: " + human_name);
  if (!sensor->sendWaitAck(reg))
  {
    throw std::runtime_error("Command to device failed.");
  }
}


/**
 * Send configuration messages to the GP9, critically, to turn on the value outputs
 * which we require, and inject necessary configuration parameters.
 */
void GP9::configureSensor(gp9::Comms* sensor)
{
  gp9::Registers r;

    uint32_t comm_reg = (BAUD_115200 << COM_BAUD_START);
    r.communication.set(0, comm_reg);
    if (!sensor->sendWaitAck(r.comrate2))
    {
      throw std::runtime_error("Unable to set CREG_COM_SETTINGS.");
    }

    uint32_t raw_rate = (0 << RATE2_ALL_RAW_START);
    r.comrate2.set(0, raw_rate);
    if (!sensor->sendWaitAck(r.comrate2))
    {
      throw std::runtime_error("Unable to set CREG_COM_RATES2.");
    }

    uint32_t proc_rate = (20 << RATE4_ALL_PROC_START);
    r.comrate4.set(0, proc_rate);
    if (!sensor->sendWaitAck(r.comrate4))
    {
      throw std::runtime_error("Unable to set CREG_COM_RATES4.");
    }

    uint32_t misc_rate = (20 << RATE5_EULER_START) | (0 << RATE5_QUAT_START);
    r.comrate5.set(0, misc_rate);
    if (!sensor->sendWaitAck(r.comrate5))
    {
      throw std::runtime_error("Unable to set CREG_COM_RATES5.");
    }

    uint32_t health_rate = (5 << RATE6_HEALTH_START);  // note:  5 gives 2 hz rate
    r.comrate6.set(0, health_rate);
    if (!sensor->sendWaitAck(r.comrate6))
    {
      throw std::runtime_error("Unable to set CREG_COM_RATES6.");
    }


  // Options available using parameters)
  // uint32_t misc_config_reg = 0;  // initialize all options off

  // Optionally disable mag updates in the sensor's EKF.
  /*
  bool mag_updates;
  ros::param::param<bool>("~mag_updates", mag_updates, true);
  if (mag_updates)
  {
    misc_config_reg |= MAG_UPDATES_ENABLED;
  }
  else
  {
    ROS_WARN("Excluding magnetometer updates from EKF.");
  }

  // Optionally enable quaternion mode .
  bool quat_mode;
  ros::param::param<bool>("~quat_mode", quat_mode, true);
  if (quat_mode)
  {
    misc_config_reg |= QUATERNION_MODE_ENABLED;
  }
  else
  {
    ROS_WARN("Excluding quaternion mode.");
  }

  r.misc_config.set(0, misc_config_reg);
  if (!sensor->sendWaitAck(r.misc_config))
  {
    throw std::runtime_error("Unable to set CREG_MISC_SETTINGS.");
  }

  // Optionally disable performing a zero gyros command on driver startup.
  bool zero_gyros;
  ros::param::param<bool>("~zero_gyros", zero_gyros, true);
  if (zero_gyros) sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");
  */
}


// bool GP9::handleResetService(gp9::Comms* sensor,
//     const gp9::Reset::Request& req, const gp9::Reset::Response& resp)
// {
//   gp9::Registers r;
//   if (req.zero_gyros) sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");
//   if (req.reset_ekf) sendCommand(sensor, r.cmd_reset_ekf, "reset EKF");
//   if (req.set_mag_ref) sendCommand(sensor, r.cmd_set_mag_ref, "set magnetometer reference");
//   return true;
// }

/**
 * Uses the register accessors to grab data from the IMU, and populate
 * the ROS messages which are output.
 */
void GP9::publishMsgs(gp9::Registers& r)
{
  // static ros::Publisher imu_pub = n->advertise<sensor_msgs::Imu>("imu/data", 1, false);
  // static ros::Publisher mag_pub = n->advertise<geometry_msgs::Vector3Stamped>("imu/mag", 1, false);
  // static ros::Publisher rpy_pub = n->advertise<geometry_msgs::Vector3Stamped>("imu/rpy", 1, false);
  // static ros::Publisher temp_pub = n->advertise<std_msgs::Float32>("imu/temperature", 1, false);

  // if (imu_pub.getNumSubscribers() > 0)
  // {
  //   sensor_msgs::Imu imu_msg;
  //   imu_msg.header = header;

  //   // IMU outputs [w,x,y,z], convert to [x,y,z,w] & transform to ROS axes
  //   imu_msg.orientation.x =  r.quat.get_scaled(1);
  //   imu_msg.orientation.y = -r.quat.get_scaled(2);
  //   imu_msg.orientation.z = -r.quat.get_scaled(3);
  //   imu_msg.orientation.w = r.quat.get_scaled(0);

  //   // Covariance of attitude.  set to constant default or parameter values
  //   imu_msg.orientation_covariance[0] = covar[0];
  //   imu_msg.orientation_covariance[1] = covar[1];
  //   imu_msg.orientation_covariance[2] = covar[2];
  //   imu_msg.orientation_covariance[3] = covar[3];
  //   imu_msg.orientation_covariance[4] = covar[4];
  //   imu_msg.orientation_covariance[5] = covar[5];
  //   imu_msg.orientation_covariance[6] = covar[6];
  //   imu_msg.orientation_covariance[7] = covar[7];
  //   imu_msg.orientation_covariance[8] = covar[8];

  //   // Angular velocity.  transform to ROS axes
  //   imu_msg.angular_velocity.x =  r.gyro.get_scaled(0);
  //   imu_msg.angular_velocity.y = -r.gyro.get_scaled(1);
  //   imu_msg.angular_velocity.z = -r.gyro.get_scaled(2);

  //   // Linear accel.  transform to ROS axes
  //   imu_msg.linear_acceleration.x =  r.accel.get_scaled(0);
  //   imu_msg.linear_acceleration.y = -r.accel.get_scaled(1);
  //   imu_msg.linear_acceleration.z = -r.accel.get_scaled(2);

  //   imu_pub.publish(imu_msg);
  // }

  // Magnetometer.  transform to ROS axes
  // if (mag_pub.getNumSubscribers() > 0)
  // {
  //   geometry_msgs::Vector3Stamped mag_msg;
  //   mag_msg.header = header;
  //   mag_msg.vector.x =  r.mag.get_scaled(0);
  //   mag_msg.vector.y = -r.mag.get_scaled(1);
  //   mag_msg.vector.z = -r.mag.get_scaled(2);
  //   mag_pub.publish(mag_msg);
  // }

  // Euler attitudes.  transform to ROS axes
  m_Comms.Notify("GP9_Roll", r.euler.get_scaled(0), MOOSTime());
  m_Comms.Notify("GP9_Pitch", r.euler.get_scaled(1), MOOSTime());
  m_Comms.Notify("GP9_Yaw", r.euler.get_scaled(2), MOOSTime());

  // Temperature
  m_Comms.Notify("GP9_Temp1", r.temperature1.get_scaled(0), MOOSTime());

  m_Comms.Notify("GP9_Lat", r.latitude.get_scaled(0), MOOSTime());
  m_Comms.Notify("GP9_Lon", r.longitude.get_scaled(0), MOOSTime());
  m_Comms.Notify("GP9_Speed", r.gps_speed.get_scaled(0), MOOSTime());
}
