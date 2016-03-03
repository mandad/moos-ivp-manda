#include "../PathPlan.h"
#include "catch.hpp"
//#include <stringstream>

template <class T>
std::string ListString(const T& print_list) {
  std::stringstream list_els;
  for (const auto& element : print_list) {
    list_els << element << " ";
  }
  return list_els.str();
}

std::string PrintPath(std::list<EPoint> print_list) {
  std::stringstream list_els;
  for (const auto& element : print_list) {
    list_els << "(" << element.transpose() << ") ";
  }
  return list_els.str();
}

TEST_CASE("Test Index selection") {
  std::list<std::size_t> test_list;
  unsigned int num_entries(10);
  for (unsigned int i = 0; i < num_entries; i++) {
    test_list.push_back(i);
  }

  REQUIRE(test_list.size() == num_entries);

  SECTION("Selecting middle indicies") {
    std::list<std::size_t> selection_ind = {3, 4, 5};
    PathPlan::SelectIndicies(test_list, selection_ind);
    INFO("Test List: " << ListString(test_list));
    REQUIRE(test_list.size() == 3);
    REQUIRE(test_list.front() == 3);
    REQUIRE(test_list.back() == 5);
  }

  SECTION("Selecting only first index") {
    std::list<std::size_t> selection_ind = {0};
    PathPlan::SelectIndicies(test_list, selection_ind);
    INFO("Test List: " << ListString(test_list));
    REQUIRE(test_list.size() == 1);
    REQUIRE(test_list.front() == 0);
    REQUIRE(test_list.back() == 0);
  }

  SECTION("Selecting only last index") {
    std::list<std::size_t> selection_ind = {num_entries - 1};
    PathPlan::SelectIndicies(test_list, selection_ind);
    INFO("Test List: " << ListString(test_list));
    REQUIRE(test_list.size() == 1);
    REQUIRE(test_list.front() == 9);
    REQUIRE(test_list.back() == 9);
  }
}

TEST_CASE("Intersection removal") {
  SECTION("Removal from the middle") {
    std::list<EPoint> path = {EPoint(0, 2), EPoint(3, 2), EPoint(5, 0),
      EPoint(4,0), EPoint(5,2), EPoint(7, 1)};
    INFO("Path before processing: \n" << PrintPath(path));

    PathPlan::RemoveIntersects(path);
    INFO("Path after processing: \n" << PrintPath(path));

    REQUIRE(path.size() == 4);
    REQUIRE(*std::next(path.begin(), 2) == EPoint(5, 2));
  }

  SECTION("Removal of multiple segs") {
    std::list<EPoint> path = {EPoint(0, 2), EPoint(2, 2), EPoint(4, 1),
      EPoint(5, 0), EPoint(3, 0), EPoint(4, 2), EPoint(6, 2), EPoint(8, 2)};
    INFO("Path before processing: \n" << PrintPath(path));

    PathPlan::RemoveIntersects(path);
    INFO("Path after processing: \n" << PrintPath(path));

    REQUIRE(path.size() == 5);
    REQUIRE(*std::next(path.begin(), 2) == EPoint(4, 2));
  }

  SECTION("First seg has intersect") {
    std::list<EPoint> path = {EPoint(0, 1), EPoint(2, 0), EPoint(1, 0),
      EPoint(2, 1), EPoint(3, 1)};
    INFO("Path before processing: \n" << PrintPath(path));

    PathPlan::RemoveIntersects(path);
    INFO("Path after processing: \n" << PrintPath(path));

    REQUIRE(path.size() == 3);
    REQUIRE(path.front() == EPoint(0, 1));
    REQUIRE(*std::next(path.begin(), 1) == EPoint(2, 1));
  }

  SECTION("Last seg has intersect") {
    std::list<EPoint> path = {EPoint(0, 1), EPoint(2, 1), EPoint(3, 1),
      EPoint(4, 0), EPoint(3, 0), EPoint(4, 1)};
    INFO("Path before processing: \n" << PrintPath(path));

    PathPlan::RemoveIntersects(path);
    INFO("Path after processing: \n" << PrintPath(path));

    REQUIRE(path.size() == 4);
    REQUIRE(path.back() == EPoint(4, 1));
    REQUIRE(*std::prev(path.end(), 2) == EPoint(3, 1));
  }
}
