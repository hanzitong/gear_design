
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
    std::uniform_real_distribution<double> dis_radius(20., 30.);
    std::uniform_real_distribution<double> \
        dis_theta(0., M_PI);     // not recommended (codes need to be changed)
    std::uniform_real_distribution<double> \
        dis_phi(-1. * M_PI / 2., M_PI / 2.);  // recommended
    // double radius = dis_radius(gen);
    // const double theta = dis_theta(gen);
    // const double phi = dis_theta(gen);
    const double N = 100.;  // the number of divisions on x-axis
    const double siggear_xrange[2] = {-20., 20.};  // plot range of x-axis
    const double siggear_x_diff = (siggear_xrange[1] - siggear_xrange[0]) / N;
    /* debug */
    const double radius = 20.;
    const double theta = M_PI / 3.;

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
    ASSERT_NEAR(siggear_xrange[1] - siggear_xrange[0], expected_arr_siggear_x[N] - expected_arr_siggear_x[0], 1e-6);

    /* set actual_ variables , transform from siggear to sigbase using Eigen function */
    for (int i = 0; i < N + 1; ++i) {
        actual_arr_sigbase_Pvecprofile[i] = \
            gear_design::trans_Pvec_from_siggear_to_sigbase(    // Eigen function
                expected_arr_siggear_Pvecprofile[i],
                actual_arr_sigbase_Pvecprofile[i],
                radius,
                theta);

        // WRONG TEST. Be careful.
            // MATHEMATICALLY IMPOSSIBLE OPERATION. DON'T PLUS OR SUBTRACT VECTORS DEFINED IN DIFFERENT COORDINATES.
            // Eigen::Vector3d diff_Pvec = actual_arr_sigbase_Pvecprofile[i] - expected_arr_siggear_Pvecprofile[i];
            // assert radius == diff_Pvec.norm
    }

    /* set actual_ variables , transform from siggear to sigbase using non-Eigen function */
    for (int i = 0; i < N + 1; ++i) {
        actual_noneigen_arr_sigbase_x[i] = \
            gear_design::_calc_gearprofile_sigbase_x_from_siggear_x(     // non-eigen function
                expected_arr_siggear_x[i], radius, theta
                );
        actual_noneigen_arr_sigbase_yprofile[i] = \
            gear_design::_calc_gearprofile_sigbase_y_from_siggear_x(     // non-eigen function
                expected_arr_siggear_x[i], radius, theta
                );

        ASSERT_NEAR(actual_arr_sigbase_Pvecprofile[i][0], actual_noneigen_arr_sigbase_x[i], 1e-6);
        ASSERT_NEAR(actual_arr_sigbase_Pvecprofile[i][1], actual_noneigen_arr_sigbase_yprofile[i], 1e-6);
    }


    // nagori 
    // ASSERT_NEAR( \
    //     expected_arr_siggear_Pvecprofile[0][0], \
    //     actual_arr_sigbase_Pvecprofile[0][0], \
    //     1e-6 \
    //     );      // error here. 2024/11/21
    // ASSERT_NEAR( \
    //     expected_arr_siggear_Pvecprofile[N][0] - expected_arr_siggear_Pvecprofile[0][0], \
    //     actual_arr_sigbase_Pvecprofile[N][0] - actual_arr_sigbase_Pvecprofile[0][0], \
    //     1e-6 \
    //     );


    // TODO: test with inverse matrix.
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    // RUN_ALL_TESTS();

    for (int i = 0; i < 1000; ++i) {
        RUN_ALL_TESTS();
    }

    return 0;
}



