
#include <random>
#include <cassert>
#include <cmath>
#include <gtest/gtest.h>

// To include Eigen, set /usr/include/eigen3 to include-PATH (g++ -I/usr/include/eigen3).
#include <Eigen/Core>
#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)

#include "../include/gear_design/coordinates.hpp"


/*----------------------------------------------------------------------
    NAME CONVENTION:  
    * expected_: the value it is expected to be. it is difenitly ture.
    * actual_: the calculated actual value. it could be false value.
    * the word "gearprofile" stands for "y-coordinate" at the coordinate.
-----------------------------------------------------------------------*/

TEST(SolutionTest, TransformGearBase) {

    /* random test condition */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_radius(10., 50.);
    std::uniform_real_distribution<double> dis_phi(-1. * M_PI / 2., M_PI / 2.);
    double radius = dis_radius(gen);
    const double phi = dis_phi(gen);
    const double N = 100.;  // the number of divisions on x-axis
    const double siggear_xrange[2] = {-20., 20.};  // plot range of x-axis
    const double siggear_x_diff = (siggear_xrange[1] - siggear_xrange[0]) / N;

    /* fixed test condition */
    // const double radius = 20.;
    // const double phi = M_PI / 3.;

    /* declare expected_ & actual_ variables */
    std::vector<double> expected_arr_siggear_x(N + 1, 0.);
    std::vector<double> expected_arr_siggear_yprofile(N + 1, 0.);
    std::vector<Eigen::Vector3d> expected_arr_siggear_Pvecprofile(N + 1, Eigen::Vector3d::Zero());
    std::vector<double> actual_arr_sigbase_x(N + 1, 0.);
    std::vector<double> actual_arr_sigbase_yprofile(N + 1, 0.);
    std::vector<Eigen::Vector3d> actual_arr_sigbase_Pvecprofile(N+1, Eigen::Vector3d::Zero());
    std::vector<double> actual_noneigen_arr_sigbase_x(N + 1, 0.);
    std::vector<double> actual_noneigen_arr_sigbase_yprofile(N + 1, 0.);
    std::vector<double> actual_arr_siggear_x(N + 1, 0.);
    std::vector<double> actual_arr_siggear_yprofile(N + 1, 0.);
    std::vector<Eigen::Vector3d> actual_arr_siggear_Pvecprofile(N + 1, Eigen::Vector3d::Zero());

    /* set expected_ variables */
    for (int i = 0; i < N + 1; ++i) {
        expected_arr_siggear_x[i] = siggear_xrange[0] + siggear_x_diff * i;
        expected_arr_siggear_yprofile[i] = \
            gear_design::quad_calc_siggear_yprofile_from_siggear_x(expected_arr_siggear_x[i]);
        expected_arr_siggear_Pvecprofile[i] << expected_arr_siggear_x[i],
                                               expected_arr_siggear_yprofile[i],
                                               1.;
    }
    /* test if xrange are set correctly */
    ASSERT_NEAR(siggear_xrange[1] - siggear_xrange[0], expected_arr_siggear_x[N] - expected_arr_siggear_x[0], 1e-6);

    /* set actual_ variables , transform from siggear to sigbase using non-Eigen function */
    for (int i = 0; i < N + 1; ++i) {
        actual_arr_sigbase_Pvecprofile[i] = \
            gear_design::trans_Pvec_from_siggear_to_sigbase(    // Eigen function
                expected_arr_siggear_Pvecprofile[i],
                actual_arr_sigbase_Pvecprofile[i],
                radius,
                phi   // trans_Pvec_from_siggear_to_sigbase()'s default direction is CLOCKWISE
            );

        actual_arr_siggear_Pvecprofile[i] = \
            gear_design::trans_Pvec_from_sigbase_to_siggear(    // Eigen function
                actual_arr_sigbase_Pvecprofile[i],
                actual_arr_siggear_Pvecprofile[i],
                radius,
                phi
            );
        /* test transformation both direction */
        ASSERT_NEAR(expected_arr_siggear_Pvecprofile[i][0], actual_arr_siggear_Pvecprofile[i][0], 1e-6);
    }

    /* Eigen usage test (not difinitly necessary) */
    for (int j = 0; j < N; ++j) {
        actual_noneigen_arr_sigbase_x[j] = \
            gear_design::_calc_gearprofile_sigbase_x_from_siggear_x(     // non-eigen function
                expected_arr_siggear_x[j],
                radius,
                phi
            );
        actual_noneigen_arr_sigbase_yprofile[j] = \
            gear_design::_calc_gearprofile_sigbase_y_from_siggear_x(     // non-eigen function
                expected_arr_siggear_x[j],
                radius,
                phi
            );

        ASSERT_NEAR(actual_arr_sigbase_Pvecprofile[j][0], actual_noneigen_arr_sigbase_x[j], 1e-6);
        ASSERT_NEAR(actual_arr_sigbase_Pvecprofile[j][1], actual_noneigen_arr_sigbase_yprofile[j], 1e-6);
    }
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    for (int i = 0; i < 5000; ++i) {
        RUN_ALL_TESTS();
    }

    return 0;
}



