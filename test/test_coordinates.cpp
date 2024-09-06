
#include <random>
#include <cassert>
#include <cmath>
#include <gtest/gtest.h>
#include "../include/gear_design/coordinates.hpp"
#include <Eigen/Core>       // To include it, set /usr/include/eigen3 to include-PATH (g++ -I/usr/include/eigen3).

/*---------------------------------------------
    naming convention
    * _expected: the value expected to be
    * _actual: the calculated actual value
----------------------------------------------*/

TEST(SolutionTest, TransformGearBase) {
    /*
    UNIT TEST
    */

    /* set situation */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_r(20., 30.);
    double radius = dis_r(gen);
    std::uniform_real_distribution<double> dis_theta(0., M_PI);
    double theta = dis_theta(gen);
    // std::uniform_real_distribution<double> dis_phi(-1. * M_PI / 2., M_PI / 2.);
    // double phi = dis_theta(gen);

    /* set test condition */
    const double N = 100.;  // N: the number of splitting
    const double siggear_x_range[2] = {-20., 20.};
    const double siggear_x_diff = (siggear_x_range[1] - siggear_x_range[0]) / N;
    std::vector<double> gearprofile_siggear_x(N, 0.);
    for (int i = 0; i < N; ++i) {
        gearprofile_siggear_x[i] = siggear_x_range[0] + siggear_x_diff * i;
    }
    Eigen::Vector3d expected_gearprofile_siggear_Pvec = Eigen::Vector3d::Zero();
    Eigen::Vector3d expected_gearprofile_sigbase_Pvec = Eigen::Vector3d::Zero();

    /* test for each siggear_f_gearprofile_x */
    for (int i = 0; i < N; ++i) {
        double actual_gearprofile_sigbase_x = \
            gear_design::calc_gearprofile_sigbase_x_from_siggear_x(siggear_gearprofile_x[i], radius, theta);
        double actual_gearprofile_sigbase_y = \
            gear_design::calc_gearprofile_sigbase_y_from_siggear_x(siggear_gearprofile_x[i], radius, theta);

        /* To transform profile_vector from siggear to sigbase, use siggear_f_gearprofile_vector which uses Eigen */
        gearprofile_siggear_Pvec << gearprofile_siggear_x[i],
                                        gear_design::siggear_f_gearprofile(siggear_f_gearprofile_x[i]),
                                        1.;
        expected_gearprofile_sigbase_Pvec = \
            // gear_design::calc_gearprofile_sigbase_Pvec_from_siggear_Pvec(
            gear_design::trans_Pvec_from_siggear_to_sigbase(
                expected_gearprofile_siggear_Pvec,
                expected_gearprofile_sigbase_Pvec,
                radius,
                theta);

        /* test */
        ASSERT_NEAR(expected_gearprofile_sigbase_vec[0], actual_gearprofile_sigbase_x, 1e-6);
        ASSERT_NEAR(expected_gearprofile_sigbase_vec[1], actual_gearprofile_sigbase_y, 1e-6);
    }


    // TODO: test with inverse matrix.
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    for (int i = 0; i < 10000; ++i) {
        RUN_ALL_TESTS();
    }

    return 0;
}



