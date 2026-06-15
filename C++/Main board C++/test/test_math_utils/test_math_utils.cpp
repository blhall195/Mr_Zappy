#include "math_utils.h"
#include <unity.h>

void setUp() {}
void tearDown() {}

static void assertNearlyEqual(float expected, float actual, float epsilon = 0.001f) {
    TEST_ASSERT_FLOAT_WITHIN(epsilon, expected, actual);
}

// ── degreesToRadians / radiansToDegrees ─────────────────────────────

void test_degreesToRadians() {
    assertNearlyEqual(0.0f,          degreesToRadians(0.0f));
    assertNearlyEqual((float)M_PI,   degreesToRadians(180.0f));
    assertNearlyEqual((float)M_PI/2, degreesToRadians(90.0f));
    assertNearlyEqual(-(float)M_PI,  degreesToRadians(-180.0f));
}

void test_radiansToDegrees() {
    assertNearlyEqual(0.0f,   radiansToDegrees(0.0f));
    assertNearlyEqual(180.0f, radiansToDegrees((float)M_PI));
    assertNearlyEqual(90.0f,  radiansToDegrees((float)M_PI / 2));
    assertNearlyEqual(-90.0f, radiansToDegrees(-(float)M_PI / 2));
}

// ── wrapTo180 ───────────────────────────────────────────────────────

void test_wrapTo180_normal() {
    assertNearlyEqual(10.0f,  wrapTo180(10.0f));
    assertNearlyEqual(-10.0f, wrapTo180(-10.0f));
    assertNearlyEqual(0.0f,   wrapTo180(0.0f));
}

void test_wrapTo180_crossing_boundary() {
    // The previously broken case: 10 - 350 should be 20, not -340
    assertNearlyEqual(20.0f,  wrapTo180(10.0f - 350.0f));
    assertNearlyEqual(-20.0f, wrapTo180(350.0f - 10.0f));
}

void test_wrapTo180_boundary_values() {
    assertNearlyEqual(-180.0f, wrapTo180(180.0f));
    assertNearlyEqual(-180.0f, wrapTo180(-180.0f));
    assertNearlyEqual(-180.0f, wrapTo180(540.0f));
}

void test_wrapTo180_large_values() {
    assertNearlyEqual(10.0f,  wrapTo180(370.0f));
    assertNearlyEqual(-10.0f, wrapTo180(-370.0f));
    assertNearlyEqual(10.0f,  wrapTo180(730.0f));
}

// ── wrapTo360 ───────────────────────────────────────────────────────

void test_wrapTo360_normal() {
    assertNearlyEqual(90.0f,  wrapTo360(90.0f));
    assertNearlyEqual(0.0f,   wrapTo360(0.0f));
    assertNearlyEqual(359.0f, wrapTo360(359.0f));
}

void test_wrapTo360_negative() {
    assertNearlyEqual(350.0f, wrapTo360(-10.0f));
    assertNearlyEqual(180.0f, wrapTo360(-180.0f));
}

void test_wrapTo360_overflow() {
    assertNearlyEqual(10.0f, wrapTo360(370.0f));
    assertNearlyEqual(10.0f, wrapTo360(730.0f));
    assertNearlyEqual(350.0f, wrapTo360(-370.0f));
}

// ── circularDiff ────────────────────────────────────────────────────

void test_circularDiff_normal() {
    assertNearlyEqual(10.0f, circularDiff(20.0f, 10.0f));
    assertNearlyEqual(10.0f, circularDiff(10.0f, 20.0f));
    assertNearlyEqual(0.0f,  circularDiff(45.0f, 45.0f));
}

void test_circularDiff_crossing_zero() {
    assertNearlyEqual(20.0f, circularDiff(10.0f, 350.0f));
    assertNearlyEqual(20.0f, circularDiff(350.0f, 10.0f));
}

void test_circularDiff_opposite() {
    assertNearlyEqual(180.0f, circularDiff(0.0f, 180.0f));
    assertNearlyEqual(180.0f, circularDiff(90.0f, 270.0f));
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_degreesToRadians);
    RUN_TEST(test_radiansToDegrees);

    RUN_TEST(test_wrapTo180_normal);
    RUN_TEST(test_wrapTo180_crossing_boundary);
    RUN_TEST(test_wrapTo180_boundary_values);
    RUN_TEST(test_wrapTo180_large_values);

    RUN_TEST(test_wrapTo360_normal);
    RUN_TEST(test_wrapTo360_negative);
    RUN_TEST(test_wrapTo360_overflow);

    RUN_TEST(test_circularDiff_normal);
    RUN_TEST(test_circularDiff_crossing_zero);
    RUN_TEST(test_circularDiff_opposite);

    return UNITY_END();
}
