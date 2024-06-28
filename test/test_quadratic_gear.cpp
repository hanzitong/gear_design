

#include <random>
#include <cassert>
#include <cmath>
#include <gtest/gtest.h>
#include "../include/gear_design/quadratic_gear.hpp"
#include <Eigen/Core>


TEST(SolutionTest, OnFunction) {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> dis_x(0., 10.);
    double x_input = dis_x(gen);
    std::uniform_real_distribution<double> dis_r(20., 30.);
    double r = dis_r(gen);
    std::uniform_real_distribution<double> dis_theta(10. / 180. * M_PI, M_PI / 2.);
    double theta = dis_theta(gen);
    std::uniform_real_distribution<double> dis_a(2., 15.);
    double a = dis_a(gen);
    std::uniform_real_distribution<double> dis_b(10., 20.);
    double b = dis_b(gen);

    /* check if test-condition is correct */
    // ASSERT_LE();
    // ASSERT_GE();

    /* test if y_output is on function or not */
    gear_design::GearParamQuadratic chank(r, theta, a, b);
    double y_output = gear_design::calc_sig0y_quadratic(x_input, &chank);

    Eigen::Matrix3d sig0_T_sig1;
    // sig0_T_sig1 = std::cos(theta_cutter), std::sin(theta_cutter), 
    // ASSERT_NEAR();

}


int main(int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);

    for(int i = 0; i < 10000; ++i){
        RUN_ALL_TESTS();
    }

    return 0;
}


