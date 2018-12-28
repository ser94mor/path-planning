//
// Created by aoool on 11.12.18.
//

#include <catch.hpp>

#include "circular_unsigned_double_t.hpp"

TEST_CASE("circular_unsigned_double_t::operator+", "[circular_unsigned_double_t]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.5);
  REQUIRE( circular_unsigned_double_t{1.0} + circular_unsigned_double_t{2.0} == Approx(3.0) );
  REQUIRE( circular_unsigned_double_t{2.0} + circular_unsigned_double_t{1.0} == Approx(3.0) );

  circular_unsigned_double_t::SetGlobalMaxValue(3.0);
  REQUIRE( circular_unsigned_double_t{1.0} + circular_unsigned_double_t{2.0} == Approx(0.0) );
  REQUIRE( circular_unsigned_double_t{2.0} + circular_unsigned_double_t{1.0} == Approx(0.0) );

  circular_unsigned_double_t::SetGlobalMaxValue(2.5);
  REQUIRE( circular_unsigned_double_t{1.0} + circular_unsigned_double_t{2.0} == Approx(0.5) );
  REQUIRE( circular_unsigned_double_t{2.0} + circular_unsigned_double_t{1.0} == Approx(0.5) );
}

TEST_CASE("circular_unsigned_double_t::operator-", "[circular_unsigned_double_t]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.0);
  REQUIRE( circular_unsigned_double_t{1.0} - circular_unsigned_double_t{1.0} == Approx(0.0) );
  REQUIRE( circular_unsigned_double_t{1.0} - circular_unsigned_double_t{1.5} == Approx(2.5) );
  REQUIRE( circular_unsigned_double_t{1.5} - circular_unsigned_double_t{1.0} == Approx(0.5) );
  REQUIRE( circular_unsigned_double_t{2.0} - circular_unsigned_double_t{1.0} == Approx(1.0) );
  REQUIRE( circular_unsigned_double_t{1.0} - circular_unsigned_double_t{2.0} == Approx(2.0) );
}

TEST_CASE("circular_unsigned_double_t::operator*", "[circular_unsigned_double_t]")
{
  // TODO: write tests
}

TEST_CASE("circular_unsigned_double_t::operator/", "[circular_unsigned_double_t]")
{
  // TODO: write tests
}

TEST_CASE("circular_unsigned_double_t::operator<", "[circular_unsigned_double_t]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.0);
  REQUIRE( !(circular_unsigned_double_t{1.0} < circular_unsigned_double_t{1.0}) );
  REQUIRE(   circular_unsigned_double_t{1.0} < circular_unsigned_double_t{1.5}  );
  REQUIRE( !(circular_unsigned_double_t{2.0} < circular_unsigned_double_t{1.0}) );

  circular_unsigned_double_t::SetGlobalMaxValue(4.0);
  REQUIRE(   circular_unsigned_double_t{0.0} < circular_unsigned_double_t{2.0}  );
  REQUIRE( !(circular_unsigned_double_t{2.0} < circular_unsigned_double_t{0.0}) );
  REQUIRE(   circular_unsigned_double_t{1.0} < circular_unsigned_double_t{3.0}  );
  REQUIRE( !(circular_unsigned_double_t{3.0} < circular_unsigned_double_t{1.0}) );
}

TEST_CASE("circular_unsigned_double_t::operator<=", "[circular_unsigned_double_t]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.0);
  REQUIRE(   circular_unsigned_double_t{1.0} <= circular_unsigned_double_t{1.0}  );
  REQUIRE(   circular_unsigned_double_t{1.0} <= circular_unsigned_double_t{1.5}  );
  REQUIRE( !(circular_unsigned_double_t{2.0} <= circular_unsigned_double_t{1.0}) );

  circular_unsigned_double_t::SetGlobalMaxValue(4.0);
  REQUIRE(   circular_unsigned_double_t{0.0} <= circular_unsigned_double_t{2.0}  );
  REQUIRE( !(circular_unsigned_double_t{2.0} <= circular_unsigned_double_t{0.0}) );
  REQUIRE(   circular_unsigned_double_t{1.0} <= circular_unsigned_double_t{3.0}  );
  REQUIRE( !(circular_unsigned_double_t{3.0} <= circular_unsigned_double_t{1.0}) );
}

TEST_CASE("circular_unsigned_double_t::operator>", "[circular_unsigned_double_t]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.0);
  REQUIRE( !(circular_unsigned_double_t{1.0} > circular_unsigned_double_t{1.0}) );
  REQUIRE( !(circular_unsigned_double_t{1.0} > circular_unsigned_double_t{1.5}) );
  REQUIRE(   circular_unsigned_double_t{2.0} > circular_unsigned_double_t{1.0}  );

  circular_unsigned_double_t::SetGlobalMaxValue(4.0);
  REQUIRE( !(circular_unsigned_double_t{0.0} > circular_unsigned_double_t{2.0}) );
  REQUIRE(   circular_unsigned_double_t{2.0} > circular_unsigned_double_t{0.0}  );
  REQUIRE( !(circular_unsigned_double_t{1.0} > circular_unsigned_double_t{3.0}) );
  REQUIRE(   circular_unsigned_double_t{3.0} > circular_unsigned_double_t{1.0}  );
}

TEST_CASE("circular_unsigned_double_t::operator>=", "[circular_unsigned_double_t]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.0);
  REQUIRE(   circular_unsigned_double_t{1.0} >= circular_unsigned_double_t{1.0} );
  REQUIRE( !(circular_unsigned_double_t{1.0} >= circular_unsigned_double_t{1.5}) );
  REQUIRE(   circular_unsigned_double_t{2.0} >= circular_unsigned_double_t{1.0}  );

  circular_unsigned_double_t::SetGlobalMaxValue(4.0);
  REQUIRE( !(circular_unsigned_double_t{0.0} >= circular_unsigned_double_t{2.0}) );
  REQUIRE(   circular_unsigned_double_t{2.0} >= circular_unsigned_double_t{0.0}  );
  REQUIRE( !(circular_unsigned_double_t{1.0} >= circular_unsigned_double_t{3.0}) );
  REQUIRE(   circular_unsigned_double_t{3.0} >= circular_unsigned_double_t{1.0}  );
}
