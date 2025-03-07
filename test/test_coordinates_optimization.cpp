
#include <iostream>
#include <vector>
#include <random>
#include <cassert>
#include <gtest/gtest.h>

// #include "../include/gear_design/coordinates.hpp"
// #include "../include/gear_design/objfunctions.hpp"
#include "gear_design/coordinates.hpp"
#include "gear_design/gear_profile.hpp"


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
    // const double radius = dis_radius(gen);
    // const double phi = dis_phi(gen);
    // const double expected_siggear_x = dis_sigbase_x(gen);

    /* fixed condition */
    const double radius = 25.; // [mm]
    const double phi = 30. / 180. * M_PI;   // [rad]. phi>0: COUNTER_CLOCKWISE
    // const double expected_siggear_x = -2.;  // old

    /* declare expected_ & actual_ variables */
    // Eigen::Vector3d expected_siggear_Pvec = Eigen::Vector3d::Zero();
    // Eigen::Vector3d expected_sigbase_Pvec = Eigen::Vector3d::Zero();
    Eigen::Vector3d actual_sigbase_Pvec = Eigen::Vector3d::Zero();
    Eigen::Vector3d actual_siggear_Pvec = Eigen::Vector3d::Zero();

    const double expected_sigbase_x = -2.;   // new

    /* set actual_ */
    actual_sigbase_Pvec << expected_sigbase_x,
                           gear_design::calc_sigbase_yprofile_from_sigbase_x(expected_sigbase_x, radius, phi),
                           1.;

    actual_siggear_Pvec = \
        gear_design::trans_Pvec_from_sigbase_to_siggear(
            actual_sigbase_Pvec,
            actual_siggear_Pvec,
            radius,
            -1. * phi  // phi>0 recognizes CLOCKWISE in this function
        );

    double actual_siggear_yprofile = \
        gear_design::quad_calc_siggear_yprofile_from_siggear_x(actual_siggear_Pvec[0]);
    ASSERT_NEAR(actual_siggear_Pvec[1], actual_siggear_yprofile, 1e-6)
        << " expected_sigbase_x: " << expected_sigbase_x \
        << "\n radius: " << radius << ", phi: " << phi \
        << "\n actual_sigbase_Pvec: \n" << actual_sigbase_Pvec \
        << "\n actual_siggear_Pvec: \n" << actual_siggear_Pvec ;

    /* set expected_ */
    // expected_siggear_Pvec << expected_siggear_x,
    //                          gear_design::quad_calc_siggear_yprofile_from_siggear_x(expected_siggear_x),
    //                          1.;
    // expected_sigbase_Pvec = \
    //     gear_design::trans_Pvec_from_siggear_to_sigbase(  // this function already passed another test
    //         expected_siggear_Pvec,
    //         expected_sigbase_Pvec,
    //         radius,
    //         phi
    //         // -1 * phi
    //     );
    
    /* set actual_ sigbase_yprofile */
    // double actual_sigbase_yprofile = \
    //     gear_design::calc_sigbase_yprofile_from_sigbase_x(
    //         expected_sigbase_Pvec[0],
    //         radius,
    //         phi,
    //         // -1 * phi,
    //         &actual_sigbase_Pvec
    //     );


    /* assert sigbase_yprofile with 2-way calculation */
    // ASSERT_NEAR(expected_sigbase_Pvec[0], actual_sigbase_Pvec[0], 1e-6);    // pass
    // ASSERT_NEAR(expected_sigbase_Pvec[1], actual_sigbase_yprofile, 1e-2) \
    //     << "expected_sigbase_Pvec: \n" << expected_sigbase_Pvec \
    //     << "\n actual_sigbase_Pvec: \n" << actual_sigbase_Pvec \
    //     << ", radius: " << radius << ", phi: " << phi \
    //     << ", expected_siggear_x: " << expected_siggear_x << ", actual_";

}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    for(int i = 0; i < 10; ++i){
        RUN_ALL_TESTS();
    }

    return 0;
}

