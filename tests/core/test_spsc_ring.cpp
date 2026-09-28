#include <catch2/catch_test_macros.hpp>

#include <thread>

#include "einstar/core/spsc_ring.hpp"

using einstar::SpscRing;

TEST_CASE("spsc ring reports full instead of overwriting") {
    SpscRing<int> ring(3);
    REQUIRE(ring.try_push(1));
    REQUIRE(ring.try_push(2));
    REQUIRE(ring.try_push(3));
    REQUIRE_FALSE(ring.try_push(4));
    REQUIRE(ring.size() == 3);
    REQUIRE(ring.try_pop() == 1);
    REQUIRE(ring.try_push(4));
    REQUIRE(ring.try_pop() == 2);
    REQUIRE(ring.try_pop() == 3);
    REQUIRE(ring.try_pop() == 4);
    REQUIRE_FALSE(ring.try_pop().has_value());
}

TEST_CASE("spsc ring preserves order across threads") {
    SpscRing<int> ring(64);
    constexpr int n = 200000;
    std::jthread producer([&] {
        for (int i = 0; i < n;)
            if (ring.try_push(i)) ++i;
    });
    int expected = 0;
    while (expected < n) {
        if (auto v = ring.try_pop()) {
            REQUIRE(*v == expected);
            ++expected;
        }
    }
}
