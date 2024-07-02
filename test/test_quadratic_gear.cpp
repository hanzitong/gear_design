

#include <random>
#include <cassert>
#include <cmath>
#include <gtest/gtest.h>
#include "../include/gear_design/quadratic_gear.hpp"
#include <Eigen/Core>


TEST(SolutionTest, OnFunction) {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> dis_r(20., 30.);
    double r = dis_r(gen);
    // std::uniform_real_distribution<double> dis_theta(10. / 180. * M_PI, M_PI / 2.);
    std::uniform_real_distribution<double> dis_theta(0., M_PI);
    double theta = dis_theta(gen);
    /* calc sig1_p from sig0_p*/
    double sin = std::sin(theta);
    double cos = std::cos(theta);
    std::uniform_real_distribution<double> dis_a(-5., -1.); // changed
    // std::uniform_real_distribution<double> dis_a(5., 20.);
    double a = dis_a(gen);
    std::uniform_real_distribution<double> dis_b(5., 15.);
    double b = dis_b(gen);
    std::uniform_real_distribution<double> dis_x(0., 1.);
    if (theta <= M_PI / 2.) {
        dis_x = std::uniform_real_distribution<double>(0., (r + b) * cos);
        ASSERT_GE((r + b) * cos, 0.);
    } else if ( M_PI / 2. < theta) {
        dis_x = std::uniform_real_distribution<double>((r + b)* cos, 0.);
        ASSERT_LE((r + b) * cos, 0.);
    } else {    // theta == M_PI / 2.
        dis_x = std::uniform_real_distribution<double>(-1. * std::sqrt(-1. * b / a), std::sqrt(-1. * b / a));
    }
    double sig0x_input = dis_x(gen);

    /* check if test-condition is correct */
    // if (theta < 0) {
    //     ASSERT_();
    // } else if (theta > 0) {
    //     ASSERT();
    // } else {    // theta = M_PI / 2.
    //     ASSERT();
    // }
    // ASSERT_LE();
    // ASSERT_GE();

    /* test if y_output is on function or not */
    gear_design::GearParamQuadratic chank(r, theta, a, b);
    double sig0y_output = gear_design::calc_sig0y_quadratic(sig0x_input, &chank);
    // ↑ why return nan !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // need to plot this failure case
    Eigen::Vector3d sig0_p = Eigen::Vector3d::Zero();
    sig0_p << sig0x_input , sig0y_output , 1.d; 
    Eigen::Matrix3d sig1_T_sig0;
    sig1_T_sig0 << cos,     - 1. * sin,    r * cos, 
                   sin,     cos,           r * sin,
                   0.d,     0.d,           1.d;
    Eigen::Vector3d sig1_p = sig1_T_sig0 * sig0_p;
    double sig1y_calculated_here = a * sig1_p[0] * sig1_p[0] + b;

    ASSERT_NEAR((sig1_p[1] - sig1y_calculated_here) / sig1_p[1], 0., 1e-6) \
    << "a: " << a << ", b: " << b << ", theta: " << theta << ", sig0x_input: " << sig0x_input \
    << ", sig0y_output: " << sig0y_output << std::endl;
}


int main(int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);

    for(int i = 0; i < 10000; ++i){
        RUN_ALL_TESTS();
    }

    return 0;
}


