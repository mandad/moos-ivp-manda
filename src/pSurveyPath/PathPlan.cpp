/**
 * @file PathPlan.cpp
 * @brief Plans a path for surveying based on a recorded path and swath.
 * @author Damian Manda
 * @date 25 Feb 2016
 * @copyright MIT License
 */

#include "PathPlan.h"

PathPlan::PathPlan(RecordSwath last_swath, BoatSide side, XYPolygon op_region,
  double margin, bool restrict_to_region) : m_last_line(last_swath),
  m_max_bend_angle(60), m_restrict_asv_to_region(restrict_to_region) {

}
