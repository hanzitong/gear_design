
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


TEST(SolutionTest, OnFunction) {

    /* random condition */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_radius(10., 50.);
    std::uniform_real_distribution<double> dis_phi(-1. * M_PI / 2., M_PI / 2.);
    std::uniform_real_distribution<double> dis_sigbase_x(-2., 2.);
    const double radius = dis_radius(gen);
    const double phi = dis_phi(gen);
    const double expected_siggear_x = dis_sigbase_x(gen);

    /* fixed condition */
    // const double radius = 25.; // [mm]
    // const double phi = 30. / 180. * M_PI;   // [rad]. equal to theta=60rad
    // const double expected_siggear_x = -2.;

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
            // -1 * phi
        );
    
    /* set actual_ sigbase_yprofile */
    // gear_design::ConditionParam param(radius, phi, expected_sigbase_x);
    double actual_sigbase_yprofile = \
        gear_design::calc_sigbase_yprofile_from_sigbase_x(
            expected_sigbase_Pvec[0],
            radius,
            phi,
            // -1 * phi,
            &actual_sigbase_Pvec
        );


    /* assert sigbase_yprofile with 2-way calculation */
    // ASSERT_NEAR(expected_sigbase_Pvec[0], actual_sigbase_Pvec[0], 1e-6);    // pass
    ASSERT_NEAR(expected_sigbase_Pvec[1], actual_sigbase_Pvec[1], 1e-2);
    // ASSERT_NEAR(expected_sigbase_Pvec[1], actual_sigbase_Pvec[1], 1e-6);
    // ASSERT_NEAR(expected_sigbase_Pvec[1], actual_sigbase_yprofile, 1e-6);

}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    for(int i = 0; i < 10; ++i){
        RUN_ALL_TESTS();
    }

    return 0;
}






