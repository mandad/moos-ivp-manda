/**
 * @file PathPlan.cpp
 * @brief Plans a path for surveying based on a recorded path and swath.
 * @author Damian Manda
 * @date 25 Feb 2016
 * @copyright MIT License
 */

#include "PathPlan.h"
#include "MOOS/libMOOS/MOOSLib.h"

#define DEBUG true

PathPlan::PathPlan(const RecordSwath &last_swath, BoatSide side, XYPolygon op_region,
  double margin, bool restrict_to_region) : m_last_line(last_swath),
  m_max_bend_angle(60), m_restrict_asv_to_region(restrict_to_region),
  m_planning_side(side), m_margin(margin) {

}

XYSegList PathPlan::GenerateNextPath() {
  #if DEBUG
  MOOSTrace("\n======== Generating Next Path ========\n");
  //MOOSTrace("'Basis Points: {0}'.format(len(edge_pts)))
  #endif
  XYSegList edge_pts = m_last_line.SwathOuterPts(m_planning_side);

  if (edge_pts.size() < 2)
    return XYSegList();

  //To satisfy the compiler for now
  return edge_pts;
}

std::list<XYPt> PathPlan::SegListToXYPt(const XYSegList &to_convert) {
  std::list<XYPt> converted;
  for (unsigned int i = 0; i < to_convert.size(); i++) {
    converted.emplace_back(XYPt{to_convert.get_vx(i), to_convert.get_vy(i)});
  }
  return converted;
}

XYSegList PathPlan::XYPtToSegList(const std::list<XYPt> &to_convert) {
  XYSegList converted;
  // std::list<XYPt>::iterator i;
  for (const XYPt &i : to_convert) {
    converted.add_vertex(i.x, i.y);
  }
  return converted;
}
