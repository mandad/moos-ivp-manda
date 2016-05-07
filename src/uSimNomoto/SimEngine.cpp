/*****************************************************************/
/*    NAME: Michael Benjamin, Henrik Schmidt, and John Leonard   */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: SimEngine.cpp                                        */
/*    DATE: Mar 8th, 2005 just another day at CSAIL              */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/

#include <iostream>
#include <cmath>
#include <chrono>
#include "AngleUtils.h"
#include "SimEngine.h"

using namespace std;

#define DEBUG false

SimEngine::SimEngine() : m_rot{0}, m_iteration_num{0}, m_distribution{0.0,1.0}, 
      m_wave_dir{45}, m_sensor_noise{0,0,0}
{
  #if DEBUG
  std::cout << "SimEngine Constructor" << endl;
  #endif
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  m_rand_gen = std::default_random_engine(seed);

}

//--------------------------------------------------------------------
// Procedure: propagate

void SimEngine::propagate(NodeRecord &record, double delta_time,
			  double prior_heading, double prior_speed,
			  double drift_x, double drift_y)
{
  double speed   = (record.getSpeed() + prior_speed) / 2;

  double s = sin(degToRadians(prior_heading)) +
    sin(degToRadians(record.getHeading()));

  double c = cos(degToRadians(prior_heading)) +
    cos(degToRadians(record.getHeading()));

  double hdg_rad = atan2(s, c);

  double prev_x = record.getX();
  double prev_y = record.getY();

  double cos_ang = cos(hdg_rad);
  double sin_ang = sin(hdg_rad);

  double xdot  = (sin_ang * speed);
  double ydot  = (cos_ang * speed);

  double new_speed = hypot(xdot, ydot);
  if(speed < 0)               // Added mikerb sep1514 bugfix
    new_speed = -new_speed;

  double new_x = prev_x + (xdot * delta_time) + (drift_x * delta_time);
  double new_y = prev_y + (ydot * delta_time) + (drift_y * delta_time);
  double new_time = record.getTimeStamp() + delta_time;
  double new_sog = hypot((xdot + drift_x), (ydot + drift_y));
  double new_hog = relAng(prev_x, prev_y,  new_x, new_y);
  
  record.setSpeed(new_speed);
  record.setX(new_x);
  record.setY(new_y);
  record.setTimeStamp(new_time);
  record.setSpeedOG(new_sog);
  record.setHeadingOG(new_hog);
}

//--------------------------------------------------------------------
// Procedure: propagateDepth

void SimEngine::propagateDepth(NodeRecord& record,
			       double delta_time,
			       double elevator_angle, 
			       double buoyancy_rate,
			       double max_depth_rate, 
			       double max_depth_rate_speed)
{
  double speed = record.getSpeed();
  double prev_depth = record.getDepth();
  elevator_angle = vclip(elevator_angle, -100, 100);
  if(speed <= 0) {
    double new_depth = prev_depth + (-1 * buoyancy_rate * delta_time);
    record.setDepth(new_depth);
    record.setPitch(0.0);
  }
  else {
    double pct = 1.0;
    if(max_depth_rate_speed > 0) {
      pct = (speed / max_depth_rate_speed);
      if(pct > 1.0)
	pct = 1.0;
    }
    if(pct < 0)
      pct = -1 * sqrt(-1 * pct);
    else
      pct = sqrt(pct);
    double depth_rate = pct * max_depth_rate;
    double speed = record.getSpeed();
    double pitch_depth_rate = - sin(record.getPitch())*speed;
    double actuator_depth_rate = (elevator_angle/100) * depth_rate;
    double total_depth_rate = (-buoyancy_rate) +  pitch_depth_rate + actuator_depth_rate;

    double new_depth = prev_depth + (1 * total_depth_rate * delta_time);
    record.setDepth(new_depth);

    // Pitch added by HS 120124

    double pitch =0 ;
    if (speed > 0 && (fabs(pitch_depth_rate+actuator_depth_rate) <= speed))
      pitch = - asin((pitch_depth_rate+actuator_depth_rate)/speed);
    record.setPitch(pitch);
  }
    
  if(record.getDepth() < 0)
    record.setDepth(0);
}


//--------------------------------------------------------------------
// Procedure: propagateSpeed

void SimEngine::propagateSpeed(NodeRecord& record, const ThrustMap& tmap,
			       double delta_time, double thrust,
			       double rudder, double max_accel, 
			       double max_decel, bool wave_sim)
{
  if(delta_time <= 0)
    return;

  if(m_thrust_mode_reverse) {
    thrust = -thrust;
    rudder = -rudder;
  }

  double next_speed  = tmap.getSpeedValue(thrust);
  double prev_speed  = record.getSpeed();

  // Apply a slowing penalty proportional to the rudder/turn
  rudder = vclip(rudder, -100, 100);
  double rudder_magnitude = fabs(rudder);
  double vpct = (rudder_magnitude / 100) * 0.85;
  next_speed *= (1.0 - vpct);
  if (wave_sim && next_speed != 0) {
    double gamma = angle180(angle180(record.getHeading()) - angle180(m_wave_dir)) * M_PI/180;
    // Waves effect speed the most head on, just a guess on the scaling factor
    next_speed += m_wave_out[0] * cos(gamma) * 0.2;
  }

  if(next_speed > prev_speed) {
    double acceleration = (next_speed - prev_speed) / delta_time;
    if((max_accel > 0) && (acceleration > max_accel))
      next_speed = (max_accel * delta_time) + prev_speed;
  }

  if(next_speed < prev_speed) {
    double deceleration = (prev_speed - next_speed) / delta_time;
    if((max_decel > 0) && (deceleration > max_decel))
      next_speed = (max_decel * delta_time * -1) + prev_speed;
  }
  record.setSpeed(next_speed);
}


//--------------------------------------------------------------------
// Procedure: propagateHeading

void SimEngine::propagateHeading(NodeRecord& record,
				 double delta_time, 
				 double rudder,
				 double thrust,
				 double turn_rate,
				 double rotate_speed,
         double km_star,
         double tm_star,
         double vessel_len,
         double rudder_offset,
         bool wave_sim,
         bool noise_sim,
         double noise_magnitude)
{

  // double vessel_len = 2;
  // double km_star = 1.66;
  // double tm_star = 0.76;

  // Assumption is that the thruster and rudder are on the same
  // actuator, e.g., like the kayaks, or typical UUVs. A rotated
  // thruster contributes nothing to a turn unless the prop is 
  // moving.
  rudder = rudder + rudder_offset;
  double speed = record.getSpeed();
  if(speed == 0) 
    rudder = 0;

  if(m_thrust_mode_reverse) {
    thrust = -thrust;
    rudder = -rudder;
  }

  double km = km_star * speed / vessel_len;
  double tm = tm_star * vessel_len / speed;
  double kim = 0;

  // Even if speed is zero, need to continue on in case the 
  // torque is non-zero.
  
  rudder    = vclip(rudder, -100, 100);
  turn_rate = vclip(turn_rate, 0, 100);
  
  /*
  // Step 1: Calculate raw delta change in heading
  double delta_deg = rudder * (turn_rate/100) * delta_time;

  // Step 2: Calculate change in heading factoring thrust
  delta_deg = (1 + ((thrust-50)/50)) * delta_deg;

  // Step 3: Calculate change in heading factoring external drift
  delta_deg += (delta_time * rotate_speed);
  */

  //New version (Nomoto)
  double prev_heading = record.getHeading();
  double m_dfModelPhiDotDot = (km * (rudder + kim) - m_rot) / tm;
  m_rot += m_dfModelPhiDotDot * delta_time;
  if (wave_sim && speed != 0) {
    double gamma = angle180(angle180(prev_heading) - angle180(m_wave_dir)) * M_PI/180;
    m_rot += m_wave_out[0] * sin(2 * gamma);
  }
  if (noise_sim && speed != 0) {
    m_rot += m_sensor_noise[0] * noise_magnitude;
    #if DEBUG
    std::cout << "uSimNomoto: Adding Sensor Noise: " << m_sensor_noise[0] * noise_magnitude << endl;
    #endif
  }
  double new_heading = prev_heading + m_rot * delta_time;

  // Step 4: Calculate final new heading in the range [0,359]
  new_heading = angle360(new_heading);
  record.setHeading(new_heading);
  record.setYaw(-degToRadians(angle180(new_heading)));
}

//--------------------------------------------------------------------
// Procedure: propagateSpeedDiffMode

void SimEngine::propagateSpeedDiffMode(NodeRecord& record, const ThrustMap& tmap,
				       double delta_time, 
				       double thrust_lft,
				       double thrust_rgt, 
				       double max_accel, 
				       double max_decel)
{
  // Sanity check
  if(delta_time <= 0)
    return;

  double thrust = (thrust_lft + thrust_rgt) / 2.0;

  // Calculate a pseudo rudder position in the range of [-100, 100]
  // Since the rudder_diff is in the range [-200, 200], just divide by 2.
  double rudder = (thrust_lft - thrust_rgt) / 2;

  propagateSpeed(record, tmap, delta_time, thrust, rudder, max_accel, max_decel, 
    false);  
}


//--------------------------------------------------------------------
// Procedure: propagateHeadingDiffMode

void SimEngine::propagateHeadingDiffMode(NodeRecord& record,
					 double delta_time, 
					 double thrust_lft,
					 double thrust_rgt,
					 double turn_rate,
					 double rotate_speed)
{
  // Sanity check
  if(delta_time <= 0)
    return;

  if(m_thrust_mode_reverse) {
    double tmp = thrust_lft;
    thrust_lft = -thrust_rgt;
    thrust_rgt = tmp;
  }

  // Calculate the raw magnitude of the turn component. Since both thrusts
  // range in [-100, 100], the maximum difference is 200. The turn_mag 
  // should range between [-1, 1].
  double turn_mag = (thrust_lft - thrust_rgt) / 200;

  // double rate_mag = (turn_rate / 100);

  // Initially we ballpark that a full-thrust forward left and full-thrust
  // backward left should turn the vehicle in 10 seconds, or 36 degrees/sec
  
  double degs_per_second = 36 * turn_mag;    // [-36, 36]

  // Step 1: Calculate raw delta change in heading
  double delta_deg = degs_per_second * delta_time;

  // Step 2: Calculate change in heading factoring external drift
  delta_deg += (delta_time * rotate_speed);

  // Step 3: Calculate final new heading in the range [0,359]
  double prev_heading = record.getHeading();
  double new_heading  = angle360(delta_deg + prev_heading);
  record.setHeading(new_heading);
  record.setYaw(-degToRadians(angle180(new_heading)));
}

WaveParameters SimEngine::determineWaveParameters(NodeRecord& record, 
  double wave_height, double wave_period, double sample_T)
{
  double H = wave_height * 1/1.4; // H_s of the waves (m)
  double T = wave_period;   // Average period (s)
  double A = 172.75 * H*H / pow(T, 4);
  double B = 691 / pow(T, 4);
  double w0 = pow(4 * B / 5, 0.25);

  // angle between ship and waves
  //double wave_dir = 90; //direction of waves, deg
  double gamma = angle180(angle180(record.getHeading()) - angle180(m_wave_dir)) 
                 * M_PI/180;
  double U = record.getSpeed();
  // encouter frequency
  double we = w0 - (w0*w0 / 9.81) * U * cos(gamma);

  // damping of the filter, choses so the variance is the same as waves
  // z = 0.05 in Amerongen
  // Smaller = more regular
  double BW =  1/(10 * T) * 2 * M_PI; // hz in rads
  double Q = we / BW;
  double zeta = 1/(2 * Q);

  double zeta_sqrt = sqrt(1 - zeta*zeta);
  double we_T = we * sample_T;
  double G = -2 * zeta * we * H / zeta_sqrt;

  WaveParameters params;
  params.b1 = -G * sqrt(1 - zeta*zeta);
  params.b2 = G * exp(-zeta * we_T)*sin(zeta_sqrt * we_T + acos(zeta));
  params.a2 = -2 * exp(-zeta * we_T)*cos(zeta_sqrt * we_T);
  params.a3 = exp(-2 * zeta * we_T);
  return params;
}

void SimEngine::propagateWaveSim(NodeRecord& record, double delta_time, 
          double wave_height, double wave_period, double wave_dir, 
          double sample_T)
{
  m_wave_dir = wave_dir;
  WaveParameters p = determineWaveParameters(record, wave_height, wave_period, 
                      sample_T);

  // On first run through the function, spin up the filter
  if (m_noise.size() == 0)
    PreFillWaves(p);

  #if DEBUG
  cout << "b1: " << p.b1 << " b2: " << p.b2 << " a2: " << p.a2 << " a3 " << p.a3 << endl;
  #endif

  // Generate the noise
  m_noise.push_front(m_distribution(m_rand_gen));

  //Apply low pass filter, wc = 0.5 Hz (2 sec period)
  m_filt_noise.push_front(0.0055 * m_noise[0] + 0.0111 * m_noise[1] 
    + 0.0055 * m_noise[2] + 1.7786 * m_filt_noise[0] - 0.8008 * m_filt_noise[1]);

  // Turn the noise into waves with a bandpass filter
  m_wave_out.push_front(p.b1*m_filt_noise[0] + p.b2*m_filt_noise[1]
    - p.a2*m_wave_out[0] - p.a3*m_wave_out[1]);

  m_noise.pop_back();
  m_filt_noise.pop_back();
  m_wave_out.pop_back();

  #if DEBUG
  std::cout << "Wave Sim Propagate End, amp "  << m_wave_out[0] << endl;
  #endif
}

void SimEngine::PreFillWaves(WaveParameters params) {
  // Fill the wave array
  m_noise.clear();
  #if DEBUG
  std::cout << "Waves prefilling" << endl;
  #endif
  for (int i = 0; i < 2; i++) {
    m_noise.push_front(m_distribution(m_rand_gen));
    if (i == 0) {
      m_filt_noise.push_front(0.0201 * m_noise[0]);
      m_wave_out.push_front(params.b1*m_filt_noise[0]);
    } else if (i == 1) {
      m_filt_noise.push_front(0.0201 * m_noise[0] + 0.0402 * m_noise[1] 
        + 1.5610 * m_filt_noise[0]);
      m_wave_out.push_front(params.b1*m_filt_noise[0] + params.b2*m_filt_noise[1]
        - params.a2*m_wave_out[0]);
    }
  }
  #if DEBUG
  std::cout << "Wave Amps: " << m_wave_out[0] << "," << m_wave_out[1] << endl;
  #endif
}

void SimEngine::propagateNoiseSim(double sample_T, bool wave_sim, double fc) {
   if (m_noise.size() == 0)
      m_noise = {0,0,0};

  // High pass filter the same noise as the waves
  double wc = 2 * M_PI * fc;
  double c = cos(wc * sample_T * 0.5)/sin(wc * sample_T * 0.5);
  
  //filter coefficients
  // Second order butterworth
  double c_sq = c * c;
  double sq_2c = sqrt(2)*c;
  vector<double> n{c_sq, -2 * c_sq, c_sq};
  vector<double> d{c_sq + sq_2c + 1, -2 * (c_sq - 1), c_sq - sq_2c + 1};
  double factor = 1/d[0];
  for (int i = 0; i < 3; i++) {
    n[i] *= factor;
    d[i] *= factor;
  }

  #if DEBUG
  //std::cout << "n0: " << n[0] << " n1: " << n[1] << " n2: " << n[2] << " d0 " << d[0] << " d1 " << d[1] << " d2 " << d[2] << endl;
  #endif

  if (!wave_sim) {
    m_noise.push_front(m_distribution(m_rand_gen));
    m_noise.pop_back();
  }

  m_sensor_noise.push_front(n[0] * m_noise[0] + n[1] * m_noise[1] 
    + n[2] * m_noise[2] - d[1] * m_sensor_noise[0] - d[2] * m_sensor_noise[1]);
  m_sensor_noise.pop_back();

}