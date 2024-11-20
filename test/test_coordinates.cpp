
#include <random>
#include <cassert>
#include <cmath>
#include <gtest/gtest.h>

#include <Eigen/Core>       // To include it, set /usr/include/eigen3 to include-PATH (g++ -I/usr/include/eigen3).
#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)

#include "../include/gear_design/coordinates.hpp"


/*---------------------------------------------
    naming convention
    * _expected: the value it is expected to be. it is difenitly ture.
    * _actual: the calculated actual value. it could be false value.
----------------------------------------------*/

TEST(SolutionTest, TransformGearBase) {
    /*
    NAME CONVENTION:
    the word "gearprofile" stands for "y-coordinate" at the coordinate.

    This code tests the following operations.
    0. At siggear coordinate, define siggear_x which are array of siggear_x.
       This array is "expected_arr_siggear_x".
    1. At siggear coordinate, calculate gearprofile from "expected_arr_siggear_x"
       This array is "expected_arr_siggear_yprofile".
    2. Transfer "expected_arr_siggear_x" to sigbase coordinate.
       This array is "actual_arr_sigbase_x".
    3. At sigbase coordinate, calculate gearprofile from actual_arr_sigbase_x.
       This array is "actual_arr_sigbase_yprofile".
    4. Check ASSERT expected_siggear_x == actual_sigbase_x
       This confirms transformation from siggear to sigbase.
    5. Check ASSERT expected_arr_siggear_yprofile == actual_arr_sigbase_yprofile
       This confirms the same transformation. (optional test)
    6. Transfer "actual_arr_sigbase_yprofile" to siggear
       This array is "actual_arr_siggear_yprofile".
    7. Check ASSERT expected_arr_siggear_yprofile == actual_arr_sigbase_yprofile
       This confirms inverse transformation.
    */

    /* set basic parameters of the situation */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_r(20., 30.);
    double radius = dis_r(gen);
    std::uniform_real_distribution<double> \
        dis_theta(0., M_PI);     // not recommended (codes need to be changed)
    std::uniform_real_distribution<double> \
        dis_phi(-1. * M_PI / 2., M_PI / 2.);  // recommended
    double theta = dis_theta(gen);
    double phi = dis_theta(gen);
    const double N = 100.;  // the number of divisions on x-axis
    const double siggear_x_range[2] = {-20., 20.};  // plot range of x-axis

    /* declare expected_ & actual_ variables */
    std::vector<double> expected_arr_siggear_x(N, 0.);
    std::vector<double> expected_arr_siggear_yprofile(N, 0.);
    std::vector<double> expected_arr_siggear_Pvecprofile(N, Eigen::Vector3d::Zero());
    std::vector<double> actual_arr_sigbase_x(N, 0.);
    std::vector<double> actual_arr_sigbase_yprofile(N, 0.);
    std::vector<double> actual_arr_sigbase_Pvecprofile(N, Eigen::Vector3d::Zero());
    std::vector<double> actual_arr_siggear_x(N, 0.);
    std::vector<double> actual_arr_siggear_yprofile(N, 0.);
    std::vector<double> actual_arr_siggear_Pvecprofile(N, Eigen::Vector3d::Zero());

    /* set expected_ variables */
    const double siggear_x_diff = (siggear_x_range[1] - siggear_x_range[0]) / N;
    for (int i = 0; i < N; ++i) {
        expected_arr_siggear_x[i] = siggear_x_range[0] + siggear_x_diff * i;
        expected_arr_siggear_yprofile[i] = \
            gear_design::quad_calc_siggear_yprofile_from_siggear_x(expected_arr_siggear_x[i]);
        expected_arr_siggear_Pvecprofile[i] << expected_arr_siggear_x[i],
                                               expected_arr_siggear_yprofile[i],
                                               1.;
    }


// today here 2024/11/20



    /* set actual_ variables , transform from siggear to sigbase using non-Eigen function */
    for (int i = 0; i < N; ++i) {
        actual_arr_sigbase_x = \
            gear_design::calc_gearprofile_sigbase_x_from_siggear_x(
                expected_arr_siggear_x[i], radius, theta
                );
        actual_arr_sigbase_yprofile = \
            gear_design::calc_gearprofile_sigbase_y_from_siggear_x(
                expected_arr_siggear_yprofile[i], radius, theta
                );
    }

    /* set actual_ variables , transform from siggear to sigbase using Eigen function */
    for (int i = 0; i < N; ++i) {
        actual_arr_siggear_x << \
            arr_gearprofile_siggear_x[i],\
            gear_design::quad_profile_calc_siggear_y_from_siggear_x(siggear_f_gearprofile_x[i]),\
            1.;
        actual_arr_siggear_yprofile = \
            // gear_design::calc_gearprofile_sigbase_Pvec_from_siggear_Pvec(
            gear_design::trans_Pvec_from_siggear_to_sigbase(
                expected_gearprofile_siggear_Pvec,
                expected_gearprofile_sigbase_Pvec,
                radius,
                theta);

        /* test */
        ASSERT_NEAR(expected_arr_sigbase_vec[0], actual_arr_sigbase_x, 1e-6);
        ASSERT_NEAR(expected_arr_sigbase_vec[1], actual_arr_sigbase_yprofile, 1e-6);
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



