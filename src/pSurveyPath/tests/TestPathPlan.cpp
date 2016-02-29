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

TEST_CASE("Test Index selection") {
  std::list<unsigned int> test_list;
  unsigned int num_entries(10);
  for (unsigned int i = 0; i < num_entries; i++) {
    test_list.push_back(i);
  }

  REQUIRE(test_list.size() == num_entries);

  SECTION("Selecting middle indicies") {
    std::list<unsigned int> selection_ind = {3, 4, 5};
    PathPlan::SelectIndicies(test_list, selection_ind);
    INFO("Test List: " << ListString(test_list));
    REQUIRE(test_list.size() == 3);
    REQUIRE(test_list.front() == 3);
    REQUIRE(test_list.back() == 5);
  }

  SECTION("Selecting only first index") {
    std::list<unsigned int> selection_ind = {0};
    PathPlan::SelectIndicies(test_list, selection_ind);
    INFO("Test List: " << ListString(test_list));
    REQUIRE(test_list.size() == 1);
    REQUIRE(test_list.front() == 0);
    REQUIRE(test_list.back() == 0);
  }

  SECTION("Selecting only last index") {
    std::list<unsigned int> selection_ind = {num_entries - 1};
    PathPlan::SelectIndicies(test_list, selection_ind);
    INFO("Test List: " << ListString(test_list));
    REQUIRE(test_list.size() == 1);
    REQUIRE(test_list.front() == 9);
    REQUIRE(test_list.back() == 9);
  }
}

TEST_CASE("Intersection removal") {
  std::list<EPoint> path = {EPoint(0, 2), EPoint(3, 2), EPoint(5, 0),
    EPoint(4,0), EPoint(5,2), EPoint(7, 1)};

    INFO("Path before processing: \n" << ListString(path));

  SECTION("Removal from the middle") {
    PathPlan::RemoveIntersects(path);

    INFO("Path after processing: \n" << ListString(path));
    REQUIRE(path.size() == 4);
  }
}
