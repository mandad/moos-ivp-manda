/**
 * @file PathPlan.cpp
 * @brief Plans a path for surveying based on a recorded path and swath.
 * @author Damian Manda
 * @date 25 Feb 2016
 * @copyright MIT License
 */

#include "PathPlan.h"
#include "MOOS/libMOOS/MOOSLib.h"
#include <Eigen/Geometry>
#include <cmath>
// #include <stdexcept>
#include <iterator>

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

  for(unsigned int i = 0; i < edge_pts.size(); i++) {
    Eigen::Vector2d back_vec;
    Eigen::Vector2d forward_vec;
    bool back_vec_set = false;
    if (i > 0) {
      back_vec.x() = edge_pts.get_vx(i) - edge_pts.get_vx(i-1);
      back_vec.y() = edge_pts.get_vy(i) - edge_pts.get_vy(i-1);
      back_vec_set = true;
    }
    if (i < edge_pts.size() - 1) {
      forward_vec.x() = edge_pts.get_vx(i + 1) - edge_pts.get_vx(i);
      forward_vec.y() = edge_pts.get_vy(i + 1) - edge_pts.get_vy(i);
    } else {
      forward_vec = back_vec;
    }
    if (!back_vec_set) {
      back_vec = forward_vec;
    }

    // Average the headings
    back_vec.normalize();
    forward_vec.normalize();
    Eigen::Vector2d avg_vec = (back_vec + forward_vec) / 2;

    // Get the offset vector
    double rot_angle = 0;
    if (m_planning_side == BoatSide::Stbd) {
      rot_angle = PI;
    } else if (m_planning_side == BoatSide::Port) {
      rot_angle = -PI;
    }
    Eigen::Rotation2D<double> rot_matrix(rot_angle);
    Eigen::Vector2d offset_vec = rot_matrix * avg_vec;
    double swath_width = m_last_line.SwathWidth(m_planning_side, i);
    offset_vec *= swath_width * (1 - m_margin);

    // Get offset location and save
    Eigen::Vector2d swath_loc(edge_pts.get_vx(i), edge_pts.get_vy(i));
    m_next_path_pts.push_back(swath_loc + offset_vec);
  }

  // Next line is in opposite direction
  m_next_path_pts.reverse();

  unsigned int pre_len = m_next_path_pts.size();
  #if DEBUG
    MOOSTrace("Eliminating path intersects itself.");
  #endif
  RemoveAll(RemoveIntersects, m_next_path_pts);

  return VectorListToSegList(m_next_path_pts);
}

// void PathPlan::RemoveAll(void (&process)(std::list<Eigen::Vector2d>&),
//   std::list<Eigen::Vector2d> &path_points) {
void PathPlan::RemoveAll(const std::function<void(std::list<Eigen::Vector2d>&)> process,
std::list<Eigen::Vector2d> &path_points) {
  unsigned int pre_len = path_points.size();
  process(path_points);
  // keep repeating until no more changes
  while (path_points.size() < pre_len) {
    pre_len = path_points.size();
    process(path_points);
  }
}

void PathPlan::RemoveIntersects(std::list<Eigen::Vector2d> &path_pts) {
  std::list<Eigen::Vector2d>::iterator i = path_pts.begin();
  // while (i < std::advance(path_pts.end(), -3)) {
  //
  // }
}

// template <typename T>
// std::list<T> PathPlan::SelectIndicies(std::list<T> select_from,
//                                       std::list<unsigned int> to_select) {
//   // Make sure the indicies are well behaved
//   to_select.sort();
//   to_select.unique();
//   if (to_select.back() >= select_from.size()) {
//     throw std::out_of_range("Indices to select exceed list size.");
//   }
//
//   std::list<T>::iterator list_it = select_from.begin();
//   unsigned int i = 0;
//   for (std::list<unsigned int>::iterator select_it = to_select.begin();
//        select_it != to_select.end(); select_it++) {
//     while (*select_it != i) {
//       // This advances list_it by one
//       list_it = select_from.erase(list_it);
//       i++;
//     }
//     list_it++;
//     i++;
//   }
// }

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

XYSegList PathPlan::VectorListToSegList(const std::list<Eigen::Vector2d> &to_convert) {
  XYSegList converted;
  for (const Eigen::Vector2d &i : to_convert) {
    converted.add_vertex(i.x(), i.y());
  }
  return converted;
}

// Eigen::Vector2d PathPlan::UnitVector(Eigen::Vector2d vector_in) {
//   double mag = vector_in.norm()
// }
