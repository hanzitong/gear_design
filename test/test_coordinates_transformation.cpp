
#include <iostream>
#include <vector>
#include <random>
#include <cassert>
#include <gtest/gtest.h>

#include "../include/gear_design/coordinates.hpp"
#include "../include/gear_design/objfunctions.hpp"



TEST(SolutionTest, OnFunction) {
    /* set situation */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_r(20., 30.);
    // double radius = dis_r(gen);
    std::uniform_real_distribution<double> dis_theta(0.1, M_PI - 0.1);
    // double theta = dis_theta(gen);
    std::uniform_real_distribution<double> dis_xbase(-1., 1.);
    // double x_base = dis_xbase(gen);

    /* fix situation */
    double radius = 13;
    double theta = 60;
    double sigbase_x_expected = 3.;

    /* calculate expected */

    /* calculate actual y_base with the function to be tested */
    /*---------------------------------------------
        naming convention
        * _expected: the value expected to be
        * _actual: the calculated actual value
    ----------------------------------------------*/
    gear_design::GearParamFgear params(radius, theta, sigbase_x_expected);
    Eigen::Vector3d siggear_vec_actual_case1 = Eigen::Vector3d::Zero();
    double y_base_actual_case1 = \
        gear_design::calc_sigbase_y_gear_from_xbase(    // test function
            sigbase_x_expected,
            radius,
            theta,
            &siggear_vec_actual_case1
        );

    Eigen::Vector3d sigbase_vec_actual_case1 = \
        gear_design::trans_siggear_to_sigbase(          // this function passed another test
            siggear_vec_actual_case1,
            sigbase_vec_actual_case1,
            radius,
            theta
        );
    double sigbase_x_actual_case1 = sigbase_vec_actual_case1[0];

    /* assert y_base with 2-way calculation */
    // ASSERT_NEAR(sigbase_x_expected, sigbase_x_actual_case1, 1e-6);
    ASSERT_NEAR(sigbase_x_expected, sigbase_x_actual_case1, 1e-6);




    /* my memo !!!!!!!!!!!!!!!!!!!!!!!
        gear_design::calc_sigbase_y_gear_from_xbase() is not correctly implemented now.
        I need to conduct numerical analysis 2 times
    */


}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    for(int i = 0; i < 10000; ++i){
        RUN_ALL_TESTS();
    }

    return 0;
}

