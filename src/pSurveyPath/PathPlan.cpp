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
  m_restrict_asv_to_region(restrict_to_region),
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

  // ---------- Intersections -----------
  unsigned int pre_len = m_next_path_pts.size();
  #if DEBUG
    MOOSTrace("Eliminating path intersects itself.\n");
  #endif
  RemoveAll(RemoveIntersects, m_next_path_pts);
  #if DEBUG
    MOOSTrace("Removed %d points.\n", pre_len - m_next_path_pts.size());
    pre_len = m_next_path_pts.size();
  #endif

  // ---------- Bends -----------
  #if DEBUG
    MOOSTrace("Eliminating sharp bends.\n");
  #endif
  std::function<void(std::list<EPoint>&)> remove_func =
    std::bind(&PathPlan::RemoveBends, this, std::placeholders::_1);
  RemoveAll(remove_func, m_next_path_pts);
  #if DEBUG
    MOOSTrace("Removed %d points.\n", pre_len - m_next_path_pts.size());
    pre_len = m_next_path_pts.size();
  #endif

  return VectorListToSegList(m_next_path_pts);
}

// void PathPlan::RemoveAll(void (&process)(std::list<Eigen::Vector2d>&),
//   std::list<Eigen::Vector2d> &path_points) {
void PathPlan::RemoveAll(std::function<void(std::list<Eigen::Vector2d>&)> process,
std::list<Eigen::Vector2d> &path_points) {
  unsigned int pre_len = path_points.size();
  process(path_points);
  // keep repeating until no more changes
  while (path_points.size() < pre_len) {
    pre_len = path_points.size();
    process(path_points);
  }
}

void PathPlan::RemoveIntersects(std::list<EPoint> &path_pts) {
  // Can't be an intersection between two segments (unless collinear)
  std::cout << "Running Remove Intersects\n";
  if (path_pts.size() < 4)
    return;

  auto path_iter = path_pts.begin();
  auto last_initial_seg = std::prev(path_pts.end(), 4);
  auto back_3 = std::next(last_initial_seg);
  auto back_2  = std::next(back_3);
  // This is the actual end of the list
  auto last_test_seg = std::next(back_2);
  //auto std::prev(path_pts.end()
  unsigned int i = 0;

  // The < operator doesn't work with non-random-access iterators
  while (path_iter != last_test_seg &&
    path_iter != back_3 && path_iter != back_2) {
    // Segment to test
    Eigen::Vector2d this_seg_a = *path_iter;
    Eigen::Vector2d this_seg_b = *(++path_iter);
    #if DEBUG
    std::cout << "First Seg Point 1: " << this_seg_a.transpose() << std::endl;
    std::cout << "First Seg Point 2: " << this_seg_b.transpose() << std::endl;
    #endif

    // Test following segments in the list
    auto j = std::next(path_iter);
    auto next_non_intersect(path_iter);
    while(j != last_test_seg) {
      Eigen::Vector2d check_seg_a = *j;
      Eigen::Vector2d check_seg_b = *(++j);
      #if DEBUG
      std::cout << "Check Seg Point 1: " << check_seg_a.transpose() << std::endl;
      std::cout << "Check Seg Point 2: " << check_seg_b.transpose() << std::endl;
      #endif
      if (Intersect(this_seg_a, this_seg_b, check_seg_a, check_seg_b)) {
        next_non_intersect = j;
        std::cout << "Found Intersect!\n";
      }
    }
    // If an intersection was found, remove the elements causing it.  Otherwise
    // the iterator advances to the next element
    if (next_non_intersect != path_iter) {
      path_iter = path_pts.erase(path_iter, next_non_intersect);
    }
  }

  // Should not need to port the code adding the end points, as this removes
  // from the list instead
}

void PathPlan::RemoveBends(std::list<EPoint> &path_pts) {
  // Maybe should process in both directions, remove pts common to both
  // Or take the method with less points removed

  std::list<std::size_t> non_bend_idx = {0};
  // Need to be able to randomly access the path points
  //std::vector<EPoint> v_pts{ std::begin(path_pts), std::end(path_pts) };
  std::vector<EPoint> v_pts;
  v_pts.reserve(path_pts.size());
  std::copy(std::begin(path_pts), std::end(path_pts), std::back_inserter(path_pts));

  SegIndex this_seg = {0, 1};
  SegIndex next_seg = {1, 2};

  while (next_seg[1] < v_pts.size() - 1) {
    EPoint this_vec = v_pts[this_seg[1]] - v_pts[this_seg[0]];
    EPoint next_vec = v_pts[next_seg[1]] - v_pts[next_seg[0]];

    // Angle between this and following segment
    if (VectorAngle(this_vec, next_vec) > m_max_bend_angle) {
      // If too large, have to find the next point that makes it small enough
      // -- Method 1: Points from end of current --
      SegIndex test_seg = next_seg + SegIndex{0, 1};
      // TODO: Added this in response to an error, not sure if correct sln
      double angle1 = 500;
      double angle2 = 500;
      while (test_seg[1] < v_pts.size() - 1) {
        EPoint test_vec = v_pts[test_seg[1]] - v_pts[test_seg[0]];
        angle1 = VectorAngle(this_vec, test_vec);
        if (angle1 < m_max_bend_angle)
          break;
        test_seg = test_seg + SegIndex{0, 1};
      }
      unsigned int pts_elim1 = test_seg[1] - test_seg[0];

      // -- Method 2: Points from beginning of current --
      // check if it would be better to remove the end of "this_vec"
      // TODO: Not entirely sure why we can't use first point
      if (non_bend_idx.size() > 1) {
        EPoint test2_vec1 = VectorFromSegment(v_pts, SegIndex{
          *(std::prev(non_bend_idx.end(), 3)), this_seg[0]});
      }
    }

    //if VectorAngle()
  }

}

bool PathPlan::Intersect(EPoint A, EPoint B, EPoint C, EPoint D) {
  return CCW(A, C, D) != CCW(B, C, D) && CCW(A, B, C) != CCW(A, B, D);
}

bool PathPlan::CCW(EPoint A, EPoint B, EPoint C) {
  return (C.y() - A.y()) * (B.x() - A.x()) > (B.y() - A.y()) * (C.x() - A.x());
}

double PathPlan::VectorAngle(EPoint vector1, EPoint vector2) {
  double dotp = vector1.dot(vector2);
  double mags = vector1.norm() * vector2.norm();

  if (mags == 0)
    return 0;
  return acos(dotp/mags) * 180/PI;
}

EPoint PathPlan::VectorFromSegment(const std::vector<EPoint>& points, SegIndex segment) {
  EPoint vector;
  try {
    vector = points[segment[1]] - points[segment[0]];
  } catch (std::exception e) {
    vector = EPoint(NAN, NAN);
  }
  return vector;
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
