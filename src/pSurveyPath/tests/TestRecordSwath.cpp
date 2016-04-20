#include "../RecordSwath.h"
#include "catch.hpp"

TEST_CASE("Test Adding Records") {
  RecordSwath swath_rec(10);
  swath_rec.SetOutputSide(BoatSide::Stbd);

  REQUIRE(!swath_rec.ValidRecord());

  SECTION("Add Sequential Points") {
    swath_rec.AddRecord(15, 10, 0, 0, 90, 5); // Should be in final
    swath_rec.AddRecord(15, 10, 0, 9, 90, 5);
    swath_rec.AddRecord(15, 10, 0, 11, 90, 5);
    swath_rec.AddRecord(15, 10, 0, 22, 90, 5); // Should be in final
    swath_rec.AddRecord(15, 10, 0, 30, 90, 5); // Should be in final
    swath_rec.AddRecord(15, 10, 0, 39, 90, 5);
    REQUIRE(swath_rec.ValidRecord());

    auto widths = swath_rec.AllSwathWidths(BoatSide::Stbd);
    REQUIRE(widths.size() == 3);

    XYPoint loc = swath_rec.SwathLocation(0);
    REQUIRE(loc.get_vx() == 0);
    REQUIRE(loc.get_vy() == 0);
    loc = swath_rec.SwathLocation(2);
    REQUIRE(loc.get_vx() == 0);
    REQUIRE(loc.get_vy() == 30);
  }

  SECTION("Different Swath Widths") {
    swath_rec.AddRecord(15, 10, 0, 0, 90, 5); // Should be in final
    swath_rec.AddRecord(15, 10, 0, 6, 90, 5);
    swath_rec.AddRecord(13, 10, 0, 9, 90, 5); // Should be in final
    swath_rec.AddRecord(15, 10, 0, 12, 90, 5);
    swath_rec.AddRecord(15, 10, 0, 16, 90, 5);
    swath_rec.AddRecord(15, 10, 0, 18, 90, 5);
    swath_rec.AddRecord(12, 10, 0, 22, 90, 5); // Should be in final

    auto widths = swath_rec.AllSwathWidths(BoatSide::Stbd);
    REQUIRE(widths.size() == 3);

    double width = swath_rec.SwathWidth(BoatSide::Stbd, 1);
    REQUIRE(width == widths[1]);
    REQUIRE(width == Approx(13));
    width = swath_rec.SwathWidth(BoatSide::Stbd, 2);
    REQUIRE(width == widths[2]);
    REQUIRE(width == Approx(12));
  }

  SECTION("Last Point Saving") {
    swath_rec.AddRecord(15, 10, 0, 0, 90, 5); // Should be in final
    swath_rec.AddRecord(15, 10, 0, 6, 90, 5);
    swath_rec.AddRecord(13, 10, 0, 9, 90, 5); // Should be in final
    swath_rec.AddRecord(15, 10, 0, 12, 90, 5);
    swath_rec.AddRecord(15, 10, 0, 16, 90, 5);
    swath_rec.AddRecord(12, 10, 0, 18, 90, 5); // Should be in final
    swath_rec.AddRecord(17, 10, 0, 22, 90, 5);
    swath_rec.AddRecord(16, 10, 0, 24, 90, 5); // Should be in final after saving
    REQUIRE(swath_rec.SaveLast());

    auto widths = swath_rec.AllSwathWidths(BoatSide::Stbd);
    REQUIRE(widths.size() == 4);

    double width = swath_rec.SwathWidth(BoatSide::Stbd, 3);
    REQUIRE(width == widths[3]);
    REQUIRE(width == Approx(16));
  }
}
