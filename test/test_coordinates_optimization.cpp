
#include <iostream>
#include <vector>
#include <random>
#include <cassert>
#include <gtest/gtest.h>

// #include "../include/gear_design/coordinates.hpp"
// #include "../include/gear_design/objfunctions.hpp"
#include "gear_design/coordinates.hpp"
#include "gear_design/objfunctions.hpp"


/* naming convention ----------------------------------------------------
    naming convention
    * expected_: the value it is expected to be. it is difenitly ture.
    * actual_: the calculated actual value. it could be false value.
-----------------------------------------------------------------------*/

/* my memo !!!!!!!!!!!!!!!!!!!!!!!
    gear_design::calc_sigbase_y_gear_from_xbase() is not correctly implemented now.
    I need to conduct numerical analysis 2 times
*/

// STEP1: numerically calculate x_gear from equation1 below.
// STEP2: calculate y_base from 2 ways.
//         WAY1: use equation2 below.
//         WAY2: transform from siggear to sigbase.

TEST(SolutionTest, OnFunction) {

    /* random condition */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_radius(20., 30.);
    std::uniform_real_distribution<double> dis_phi(0.1, M_PI - 0.1);
    std::uniform_real_distribution<double> dis_xbase(-1., 1.);
    // double radius = dis_radius(gen);
    // double phi = dis_phi(gen);
    // double x_base = dis_xbase(gen);

    /* fixed condition */
    double radius = 50;
    double phi = -30;   // equal to theta=60
    double expected_siggear_x = -2.;

    /* declare expected_ & actual_ variables */
    Eigen::Vector3d expected_siggear_Pvec = Eigen::Vector3d::Zero();
    Eigen::Vector3d expected_sigbase_Pvec = Eigen::Vector3d::Zero();
    Eigen::Vector3d actual_sigbase_Pvec = Eigen::Vector3d::Zero();

    /* set expected_ */
    expected_siggear_Pvec << expected_siggear_x,
                                   gear_design::quad_calc_siggear_yprofile_from_siggear_x(expected_siggear_x),
                                   1.;
    expected_sigbase_Pvec = \
        gear_design::trans_Pvec_from_siggear_to_sigbase(  // this function already passed another test
            expected_siggear_Pvec,
            expected_sigbase_Pvec,
            radius,
            phi
        );
    
    /* set actual_ sigbase_yprofile */
    // gear_design::ConditionParam param(radius, phi, expected_sigbase_x);
    double actual_sigbase_yprofile = \
        gear_design::calc_sigbase_yprofile_from_sigbase_x(
            expected_sigbase_Pvec[0],
            radius,
            phi,
            &actual_sigbase_Pvec
        );

    // double actual_sigbase_x = actual_sigbase_Pvec[0];

    /* assert sigbase_yprofile with 2-way calculation */
    // ASSERT_NEAR(expected_siggear_x, actual_sigbase_Pvec[2], 1e-6);    // siggear_x
    // ASSERT_NEAR(expected_sigbase_Pvec[0], actual_sigbase_Pvec[0], 1e-6);
    // ASSERT_NEAR(expected_sigbase_Pvec[1], actual_sigbase_Pvec[1], 1e-6);
    ASSERT_NEAR(expected_sigbase_Pvec[1], actual_sigbase_yprofile, 1e-6);

}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // ::testing::internal::CaptureStdout();

    for(int i = 0; i < 10; ++i){
        RUN_ALL_TESTS();
    }

    // std::string output = ::testing::internal::GetCapturedStdout();
    // std::cout << output << std::endl;

    return 0;
}

