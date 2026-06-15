// Critical boundary values computed by test/verify_test_values.py using
// numpy/scipy (independent of this C++ implementation).
// Each inside/outside pair is ±0.5% of the exact critical delta.

#include "leg_checker.h"
#include <unity.h>

void setUp() {}
void tearDown() {}

// ── CartesianCoordinate::fromShot ────────────────────────────────────

void test_fromShot_north() {
    CartesianCoordinate c = CartesianCoordinate::fromShot({0.0f, 0.0f, 10.0f});
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  c.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, c.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  c.z);
}

void test_fromShot_east() {
    CartesianCoordinate c = CartesianCoordinate::fromShot({90.0f, 0.0f, 10.0f});
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, c.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  c.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  c.z);
}

void test_fromShot_vertical_up() {
    CartesianCoordinate c = CartesianCoordinate::fromShot({0.0f, 90.0f, 10.0f});
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  c.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,  c.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, c.z);
}

// ── CartesianLegChecker — guards ─────────────────────────────────────

void test_cart_null_false()           { CartesianLegChecker c(10.0f); TEST_ASSERT_FALSE(c.hasValidLeg(nullptr, 1)); }
void test_cart_zero_count_false()     { CartesianLegChecker c(10.0f); Shot s[] = {{0,0,5}}; TEST_ASSERT_FALSE(c.hasValidLeg(s,  0)); }
void test_cart_negative_count_false() { CartesianLegChecker c(10.0f); Shot s[] = {{0,0,5}}; TEST_ASSERT_FALSE(c.hasValidLeg(s, -1)); }
void test_cart_count_too_large_false(){ CartesianLegChecker c(10.0f); Shot s[9]={};          TEST_ASSERT_FALSE(c.hasValidLeg(s,  9)); }
void test_cart_single_shot_true()     { CartesianLegChecker c(10.0f); Shot s[] = {{45,30,5}};TEST_ASSERT_TRUE(c.hasValidLeg(s,  1)); }

// ── CartesianLegChecker — tol=0: only identical shots pass ──────────

void test_cart_tol0_identical_true() {
    CartesianLegChecker c(0.0f);
    Shot s[] = {{45.0f, 20.0f, 10.0f}, {45.0f, 20.0f, 10.0f}};
    TEST_ASSERT_TRUE(c.hasValidLeg(s, 2));
}

void test_cart_tol0_any_difference_false() {
    CartesianLegChecker c(0.0f);
    Shot s[] = {{0.0f, 0.0f, 5.0f}, {0.001f, 0.0f, 5.0f}};
    TEST_ASSERT_FALSE(c.hasValidLeg(s, 2));
}

// ── CartesianLegChecker — dist=0: all shots at origin, always pass ──

void test_cart_dist0_any_angle_true() {
    CartesianLegChecker c(5.0f);
    Shot s[] = {{0.0f, 0.0f, 0.0f}, {45.0f, 45.0f, 0.0f}, {180.0f, -30.0f, 0.0f}};
    TEST_ASSERT_TRUE(c.hasValidLeg(s, 3));
}

// ── CartesianLegChecker — tol=5cm ────────────────────────────────────
// Critical deltas from verify_test_values.py (inside=0.995×crit, outside=1.005×crit)

void test_cart_tol5_dist5_az_inside()    { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{0.570095f,0.0f,5.0f}};   TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol5_dist5_az_outside()   { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{0.575825f,0.0f,5.0f}};   TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol5_dist5_inc_inside()   { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,0.570095f,5.0f}};   TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol5_dist5_inc_outside()  { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,0.575825f,5.0f}};   TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol5_dist5_diag_inside()  { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{0.403120f,0.403120f,5.0f}};TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol5_dist5_diag_outside() { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{0.407171f,0.407171f,5.0f}};TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }

void test_cart_tol5_dist10_az_inside()   { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,10.0f},{0.285047f,0.0f,10.0f}};  TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol5_dist10_az_outside()  { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,10.0f},{0.287912f,0.0f,10.0f}};  TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol5_dist10_inc_inside()  { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,10.0f},{0.0f,0.285047f,10.0f}};  TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol5_dist10_inc_outside() { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,10.0f},{0.0f,0.287912f,10.0f}};  TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol5_dist10_diag_inside() { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,10.0f},{0.201559f,0.201559f,10.0f}};TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol5_dist10_diag_outside(){ CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,10.0f},{0.203584f,0.203584f,10.0f}};TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }

void test_cart_tol5_dist100_az_inside()  { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,100.0f},{0.028505f,0.0f,100.0f}}; TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol5_dist100_az_outside() { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,100.0f},{0.028791f,0.0f,100.0f}}; TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol5_dist100_inc_inside() { CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,100.0f},{0.0f,0.028505f,100.0f}}; TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol5_dist100_inc_outside(){ CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,100.0f},{0.0f,0.028791f,100.0f}}; TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol5_dist100_diag_inside(){ CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,100.0f},{0.020156f,0.020156f,100.0f}};TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol5_dist100_diag_outside(){CartesianLegChecker c(5.0f);  Shot s[] = {{0.0f,0.0f,100.0f},{0.020358f,0.020358f,100.0f}};TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }

// ── CartesianLegChecker — tol=10cm ───────────────────────────────────

void test_cart_tol10_dist5_az_inside()   { CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{1.140205f,0.0f,5.0f}};    TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol10_dist5_az_outside()  { CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{1.151664f,0.0f,5.0f}};    TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol10_dist5_inc_inside()  { CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,1.140205f,5.0f}};    TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol10_dist5_inc_outside() { CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,1.151664f,5.0f}};    TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol10_dist5_diag_inside() { CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{0.806260f,0.806260f,5.0f}};TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol10_dist5_diag_outside(){ CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{0.814363f,0.814363f,5.0f}};TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }

void test_cart_tol10_dist10_az_inside()  { CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,10.0f},{0.570095f,0.0f,10.0f}};  TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol10_dist10_az_outside() { CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,10.0f},{0.575825f,0.0f,10.0f}};  TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol10_dist10_inc_inside() { CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,10.0f},{0.0f,0.570095f,10.0f}};  TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol10_dist10_inc_outside(){ CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,10.0f},{0.0f,0.575825f,10.0f}};  TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol10_dist10_diag_inside(){ CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,10.0f},{0.403120f,0.403120f,10.0f}};TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol10_dist10_diag_outside(){CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,10.0f},{0.407171f,0.407171f,10.0f}};TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }

void test_cart_tol10_dist100_az_inside() { CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,100.0f},{0.057009f,0.0f,100.0f}}; TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol10_dist100_az_outside(){ CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,100.0f},{0.057582f,0.0f,100.0f}}; TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol10_dist100_inc_inside(){ CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,100.0f},{0.0f,0.057009f,100.0f}}; TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol10_dist100_inc_outside(){CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,100.0f},{0.0f,0.057582f,100.0f}}; TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol10_dist100_diag_inside(){CartesianLegChecker c(10.0f); Shot s[] = {{0.0f,0.0f,100.0f},{0.040312f,0.040312f,100.0f}};TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol10_dist100_diag_outside(){CartesianLegChecker c(10.0f);Shot s[] = {{0.0f,0.0f,100.0f},{0.040717f,0.040717f,100.0f}};TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }

// ── CartesianLegChecker — tol=30cm ───────────────────────────────────

void test_cart_tol30_dist5_az_inside()   { CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,5.0f},{3.421071f,0.0f,5.0f}};    TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol30_dist5_az_outside()  { CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,5.0f},{3.455454f,0.0f,5.0f}};    TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol30_dist5_inc_inside()  { CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,3.421071f,5.0f}};    TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol30_dist5_inc_outside() { CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,3.455454f,5.0f}};    TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol30_dist5_diag_inside() { CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,5.0f},{2.419426f,2.419426f,5.0f}};TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol30_dist5_diag_outside(){ CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,5.0f},{2.443742f,2.443742f,5.0f}};TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }

void test_cart_tol30_dist10_az_inside()  { CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,10.0f},{1.710343f,0.0f,10.0f}};  TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol30_dist10_az_outside() { CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,10.0f},{1.727533f,0.0f,10.0f}};  TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol30_dist10_inc_inside() { CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,10.0f},{0.0f,1.710343f,10.0f}};  TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol30_dist10_inc_outside(){ CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,10.0f},{0.0f,1.727533f,10.0f}};  TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol30_dist10_diag_inside(){ CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,10.0f},{1.209441f,1.209441f,10.0f}};TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol30_dist10_diag_outside(){CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,10.0f},{1.221596f,1.221596f,10.0f}};TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }

void test_cart_tol30_dist100_az_inside() { CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,100.0f},{0.171028f,0.0f,100.0f}}; TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol30_dist100_az_outside(){ CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,100.0f},{0.172747f,0.0f,100.0f}}; TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol30_dist100_inc_inside(){ CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,100.0f},{0.0f,0.171028f,100.0f}}; TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol30_dist100_inc_outside(){CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,100.0f},{0.0f,0.172747f,100.0f}}; TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }
void test_cart_tol30_dist100_diag_inside(){CartesianLegChecker c(30.0f); Shot s[] = {{0.0f,0.0f,100.0f},{0.120935f,0.120935f,100.0f}};TEST_ASSERT_TRUE (c.hasValidLeg(s,2)); }
void test_cart_tol30_dist100_diag_outside(){CartesianLegChecker c(30.0f);Shot s[] = {{0.0f,0.0f,100.0f},{0.122151f,0.122151f,100.0f}};TEST_ASSERT_FALSE(c.hasValidLeg(s,2)); }

// ── CartesianLegChecker — setTolerance ───────────────────────────────

void test_cart_setTolerance_updates() {
    CartesianLegChecker c(30.0f);
    Shot s[] = {{0.0f, 0.0f, 10.0f}, {1.710343f, 0.0f, 10.0f}}; // ~29.85cm
    TEST_ASSERT_TRUE(c.hasValidLeg(s, 2));
    c.setTolerance(10.0f);
    TEST_ASSERT_FALSE(c.hasValidLeg(s, 2));
}

void test_cart_setTolerance_invalid_ignored() {
    CartesianLegChecker c(10.0f);
    c.setTolerance(0.0f); // below 1cm minimum — old tolerance kept
    Shot s[] = {{0.0f, 0.0f, 10.0f}, {0.570095f, 0.0f, 10.0f}}; // ~9.95cm
    TEST_ASSERT_TRUE(c.hasValidLeg(s, 2));
}

// ── CartesianLegChecker — near-vertical, wrap-around ─────────────────

void test_cart_near_vertical_az4_az356() {
    // inc=89°: az=4° and az=356° are 8° apart as scalars but endpoints only ~2.4cm apart
    CartesianLegChecker c(5.0f);
    Shot s[] = {{4.0f, 89.0f, 10.0f}, {356.0f, 89.0f, 10.0f}};
    TEST_ASSERT_TRUE(c.hasValidLeg(s, 2));
}

void test_cart_az1_az359_wrap() {
    // 1° and 359° are 2° apart — endpoints ~17.5cm apart at 5m
    CartesianLegChecker c(20.0f);
    Shot s[] = {{1.0f, 0.0f, 5.0f}, {359.0f, 0.0f, 5.0f}};
    TEST_ASSERT_TRUE(c.hasValidLeg(s, 2));
}

// ── AngularLegChecker — guards ────────────────────────────────────────

void test_ang_null_false()            { AngularLegChecker a(1.7f); TEST_ASSERT_FALSE(a.hasValidLeg(nullptr, 1)); }
void test_ang_zero_count_false()      { AngularLegChecker a(1.7f); Shot s[] = {{0,0,5}}; TEST_ASSERT_FALSE(a.hasValidLeg(s,  0)); }
void test_ang_negative_count_false()  { AngularLegChecker a(1.7f); Shot s[] = {{0,0,5}}; TEST_ASSERT_FALSE(a.hasValidLeg(s, -1)); }
void test_ang_count_too_large_false() { AngularLegChecker a(1.7f); Shot s[9]={};         TEST_ASSERT_FALSE(a.hasValidLeg(s,  9)); }
void test_ang_single_shot_true()      { AngularLegChecker a(1.7f); Shot s[] = {{45,30,5}};TEST_ASSERT_TRUE(a.hasValidLeg(s,  1)); }

// ── AngularLegChecker — tol=0°: only identical shots pass ────────────

void test_ang_tol0_identical_true() {
    // Use az=0°, inc=0°: sin/cos are exact in float32, dot product is exactly 1.0
    AngularLegChecker a(0.0f);
    Shot s[] = {{0.0f, 0.0f, 5.0f}, {0.0f, 0.0f, 5.0f}};
    TEST_ASSERT_TRUE(a.hasValidLeg(s, 2));
}

void test_ang_tol0_any_difference_false() {
    // 1° is large enough to be detectable in float32 (0.001° is not)
    AngularLegChecker a(0.0f);
    Shot s[] = {{0.0f, 0.0f, 5.0f}, {1.0f, 0.0f, 5.0f}};
    TEST_ASSERT_FALSE(a.hasValidLeg(s, 2));
}

// ── AngularLegChecker — tol=1.7° ─────────────────────────────────────
// For az/inc single-axis: critical delta == tolerance exactly.
// For diag: critical delta = 1.20212563° (arccos(sqrt(cos(1.7°)))).

void test_ang_tol17_az_inside()    { AngularLegChecker a(1.7f); Shot s[] = {{0.0f,0.0f,5.0f},{1.691500f,0.0f,5.0f}};            TEST_ASSERT_TRUE (a.hasValidLeg(s,2)); }
void test_ang_tol17_az_outside()   { AngularLegChecker a(1.7f); Shot s[] = {{0.0f,0.0f,5.0f},{1.708500f,0.0f,5.0f}};            TEST_ASSERT_FALSE(a.hasValidLeg(s,2)); }
void test_ang_tol17_inc_inside()   { AngularLegChecker a(1.7f); Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,1.691500f,5.0f}};            TEST_ASSERT_TRUE (a.hasValidLeg(s,2)); }
void test_ang_tol17_inc_outside()  { AngularLegChecker a(1.7f); Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,1.708500f,5.0f}};            TEST_ASSERT_FALSE(a.hasValidLeg(s,2)); }
void test_ang_tol17_diag_inside()  { AngularLegChecker a(1.7f); Shot s[] = {{0.0f,0.0f,5.0f},{1.196115f,1.196115f,5.0f}};       TEST_ASSERT_TRUE (a.hasValidLeg(s,2)); }
void test_ang_tol17_diag_outside() { AngularLegChecker a(1.7f); Shot s[] = {{0.0f,0.0f,5.0f},{1.208136f,1.208136f,5.0f}};       TEST_ASSERT_FALSE(a.hasValidLeg(s,2)); }

// ── AngularLegChecker — tol=5° ───────────────────────────────────────

void test_ang_tol5_az_inside()     { AngularLegChecker a(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{4.975000f,0.0f,5.0f}};           TEST_ASSERT_TRUE (a.hasValidLeg(s,2)); }
void test_ang_tol5_az_outside()    { AngularLegChecker a(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{5.025000f,0.0f,5.0f}};           TEST_ASSERT_FALSE(a.hasValidLeg(s,2)); }
void test_ang_tol5_inc_inside()    { AngularLegChecker a(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,4.975000f,5.0f}};           TEST_ASSERT_TRUE (a.hasValidLeg(s,2)); }
void test_ang_tol5_inc_outside()   { AngularLegChecker a(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,5.025000f,5.0f}};           TEST_ASSERT_FALSE(a.hasValidLeg(s,2)); }
void test_ang_tol5_diag_inside()   { AngularLegChecker a(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{3.518974f,3.518974f,5.0f}};      TEST_ASSERT_TRUE (a.hasValidLeg(s,2)); }
void test_ang_tol5_diag_outside()  { AngularLegChecker a(5.0f);  Shot s[] = {{0.0f,0.0f,5.0f},{3.554341f,3.554341f,5.0f}};      TEST_ASSERT_FALSE(a.hasValidLeg(s,2)); }

// ── AngularLegChecker — tol=10° ──────────────────────────────────────

void test_ang_tol10_az_inside()    { AngularLegChecker a(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{9.950000f,0.0f,5.0f}};           TEST_ASSERT_TRUE (a.hasValidLeg(s,2)); }
void test_ang_tol10_az_outside()   { AngularLegChecker a(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{10.050000f,0.0f,5.0f}};          TEST_ASSERT_FALSE(a.hasValidLeg(s,2)); }
void test_ang_tol10_inc_inside()   { AngularLegChecker a(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,9.950000f,5.0f}};           TEST_ASSERT_TRUE (a.hasValidLeg(s,2)); }
void test_ang_tol10_inc_outside()  { AngularLegChecker a(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{0.0f,10.050000f,5.0f}};          TEST_ASSERT_FALSE(a.hasValidLeg(s,2)); }
void test_ang_tol10_diag_inside()  { AngularLegChecker a(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{7.044701f,7.044701f,5.0f}};      TEST_ASSERT_TRUE (a.hasValidLeg(s,2)); }
void test_ang_tol10_diag_outside() { AngularLegChecker a(10.0f); Shot s[] = {{0.0f,0.0f,5.0f},{7.115502f,7.115502f,5.0f}};      TEST_ASSERT_FALSE(a.hasValidLeg(s,2)); }

// ── AngularLegChecker — distance independence ─────────────────────────

void test_ang_distance_irrelevant() {
    // Same angle, different distances — all pass
    AngularLegChecker a(1.7f);
    Shot s0[]   = {{0.0f,0.0f,0.0f},   {1.691500f,0.0f,0.0f}};
    Shot s5[]   = {{0.0f,0.0f,5.0f},   {1.691500f,0.0f,5.0f}};
    Shot s100[] = {{0.0f,0.0f,100.0f}, {1.691500f,0.0f,100.0f}};
    TEST_ASSERT_TRUE(a.hasValidLeg(s0,   2));
    TEST_ASSERT_TRUE(a.hasValidLeg(s5,   2));
    TEST_ASSERT_TRUE(a.hasValidLeg(s100, 2));
}

// ── AngularLegChecker — near-vertical, wrap-around ───────────────────

void test_ang_near_vertical_az4_az356() {
    // inc=89°: dot-product gives ~0.14° — correctly accepts unlike naive az comparison
    AngularLegChecker a(1.7f);
    Shot s[] = {{4.0f, 89.0f, 10.0f}, {356.0f, 89.0f, 10.0f}};
    TEST_ASSERT_TRUE(a.hasValidLeg(s, 2));
}

void test_ang_az1_az359_wrap() {
    // 1° and 359° → 2° angular separation, passes at 3° tolerance
    AngularLegChecker a(3.0f);
    Shot s[] = {{1.0f, 0.0f, 5.0f}, {359.0f, 0.0f, 5.0f}};
    TEST_ASSERT_TRUE(a.hasValidLeg(s, 2));
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_fromShot_north);
    RUN_TEST(test_fromShot_east);
    RUN_TEST(test_fromShot_vertical_up);

    RUN_TEST(test_cart_null_false);
    RUN_TEST(test_cart_zero_count_false);
    RUN_TEST(test_cart_negative_count_false);
    RUN_TEST(test_cart_count_too_large_false);
    RUN_TEST(test_cart_single_shot_true);

    RUN_TEST(test_cart_tol0_identical_true);
    RUN_TEST(test_cart_tol0_any_difference_false);
    RUN_TEST(test_cart_dist0_any_angle_true);

    RUN_TEST(test_cart_tol5_dist5_az_inside);   RUN_TEST(test_cart_tol5_dist5_az_outside);
    RUN_TEST(test_cart_tol5_dist5_inc_inside);  RUN_TEST(test_cart_tol5_dist5_inc_outside);
    RUN_TEST(test_cart_tol5_dist5_diag_inside); RUN_TEST(test_cart_tol5_dist5_diag_outside);

    RUN_TEST(test_cart_tol5_dist10_az_inside);   RUN_TEST(test_cart_tol5_dist10_az_outside);
    RUN_TEST(test_cart_tol5_dist10_inc_inside);  RUN_TEST(test_cart_tol5_dist10_inc_outside);
    RUN_TEST(test_cart_tol5_dist10_diag_inside); RUN_TEST(test_cart_tol5_dist10_diag_outside);

    RUN_TEST(test_cart_tol5_dist100_az_inside);   RUN_TEST(test_cart_tol5_dist100_az_outside);
    RUN_TEST(test_cart_tol5_dist100_inc_inside);  RUN_TEST(test_cart_tol5_dist100_inc_outside);
    RUN_TEST(test_cart_tol5_dist100_diag_inside); RUN_TEST(test_cart_tol5_dist100_diag_outside);

    RUN_TEST(test_cart_tol10_dist5_az_inside);   RUN_TEST(test_cart_tol10_dist5_az_outside);
    RUN_TEST(test_cart_tol10_dist5_inc_inside);  RUN_TEST(test_cart_tol10_dist5_inc_outside);
    RUN_TEST(test_cart_tol10_dist5_diag_inside); RUN_TEST(test_cart_tol10_dist5_diag_outside);

    RUN_TEST(test_cart_tol10_dist10_az_inside);   RUN_TEST(test_cart_tol10_dist10_az_outside);
    RUN_TEST(test_cart_tol10_dist10_inc_inside);  RUN_TEST(test_cart_tol10_dist10_inc_outside);
    RUN_TEST(test_cart_tol10_dist10_diag_inside); RUN_TEST(test_cart_tol10_dist10_diag_outside);

    RUN_TEST(test_cart_tol10_dist100_az_inside);   RUN_TEST(test_cart_tol10_dist100_az_outside);
    RUN_TEST(test_cart_tol10_dist100_inc_inside);  RUN_TEST(test_cart_tol10_dist100_inc_outside);
    RUN_TEST(test_cart_tol10_dist100_diag_inside); RUN_TEST(test_cart_tol10_dist100_diag_outside);

    RUN_TEST(test_cart_tol30_dist5_az_inside);   RUN_TEST(test_cart_tol30_dist5_az_outside);
    RUN_TEST(test_cart_tol30_dist5_inc_inside);  RUN_TEST(test_cart_tol30_dist5_inc_outside);
    RUN_TEST(test_cart_tol30_dist5_diag_inside); RUN_TEST(test_cart_tol30_dist5_diag_outside);

    RUN_TEST(test_cart_tol30_dist10_az_inside);   RUN_TEST(test_cart_tol30_dist10_az_outside);
    RUN_TEST(test_cart_tol30_dist10_inc_inside);  RUN_TEST(test_cart_tol30_dist10_inc_outside);
    RUN_TEST(test_cart_tol30_dist10_diag_inside); RUN_TEST(test_cart_tol30_dist10_diag_outside);

    RUN_TEST(test_cart_tol30_dist100_az_inside);   RUN_TEST(test_cart_tol30_dist100_az_outside);
    RUN_TEST(test_cart_tol30_dist100_inc_inside);  RUN_TEST(test_cart_tol30_dist100_inc_outside);
    RUN_TEST(test_cart_tol30_dist100_diag_inside); RUN_TEST(test_cart_tol30_dist100_diag_outside);

    RUN_TEST(test_cart_setTolerance_updates);
    RUN_TEST(test_cart_setTolerance_invalid_ignored);
    RUN_TEST(test_cart_near_vertical_az4_az356);
    RUN_TEST(test_cart_az1_az359_wrap);

    RUN_TEST(test_ang_null_false);
    RUN_TEST(test_ang_zero_count_false);
    RUN_TEST(test_ang_negative_count_false);
    RUN_TEST(test_ang_count_too_large_false);
    RUN_TEST(test_ang_single_shot_true);

    RUN_TEST(test_ang_tol0_identical_true);
    RUN_TEST(test_ang_tol0_any_difference_false);

    RUN_TEST(test_ang_tol17_az_inside);   RUN_TEST(test_ang_tol17_az_outside);
    RUN_TEST(test_ang_tol17_inc_inside);  RUN_TEST(test_ang_tol17_inc_outside);
    RUN_TEST(test_ang_tol17_diag_inside); RUN_TEST(test_ang_tol17_diag_outside);

    RUN_TEST(test_ang_tol5_az_inside);   RUN_TEST(test_ang_tol5_az_outside);
    RUN_TEST(test_ang_tol5_inc_inside);  RUN_TEST(test_ang_tol5_inc_outside);
    RUN_TEST(test_ang_tol5_diag_inside); RUN_TEST(test_ang_tol5_diag_outside);

    RUN_TEST(test_ang_tol10_az_inside);   RUN_TEST(test_ang_tol10_az_outside);
    RUN_TEST(test_ang_tol10_inc_inside);  RUN_TEST(test_ang_tol10_inc_outside);
    RUN_TEST(test_ang_tol10_diag_inside); RUN_TEST(test_ang_tol10_diag_outside);

    RUN_TEST(test_ang_distance_irrelevant);
    RUN_TEST(test_ang_near_vertical_az4_az356);
    RUN_TEST(test_ang_az1_az359_wrap);

    return UNITY_END();
}
