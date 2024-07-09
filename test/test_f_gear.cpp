
// g++ 

#include <random>
#include <cassert>
#include <cmath>
#include <gtest/gtest.h>
#include "../include/gear_design/f_gear.hpp"
#include <Eigen/Core>       // To include it, set /usr/include/eigen3 to include-PATH (g++ -I/usr/include/eigen3).


TEST(SolutionTest, SameResult) {
    /* set test conditions */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_r(20., 30.);
    double radius = dis_r(gen);
    std::uniform_real_distribution<double> dis_theta(0., M_PI);
    double theta = dis_theta(gen);

    /* set test condition of axis-siggear_gearprofile_x */
    const double N = 100.;
    const double siggear_x_range[2] = {-2., 2.};
    const double siggear_x_diff = (siggear_x_range[1] - siggear_x_range[0]) / N;
    std::vector<double> siggear_f_gearprofile_x(N, 0.);
    for (int i = 0; i < N; ++i) {
        siggear_f_gearprofile_x[i] = siggear_x_range[0] + siggear_x_diff * i;
    }

    /* check if test-condition is correct */
    // ASSERT_LE();
    // ASSERT_GE();

    /* using x,y, function to calculate. */
    std::vector<double> test_sigbase_f_gearprofile_x(N, 0.);
    std::vector<double> test_sigbase_f_gearprofile_y(N, 0.);
    for (int i = 0; i < N; ++i) {
        test_sigbase_f_gearprofile_x[i] = \
            gear_design::calc_sigbase_f_gearprofile_x_coordinate(siggear_f_gearprofile_x[i], radius, theta);
        test_sigbase_f_gearprofile_y[i] = \
            gear_design::calc_sigbase_f_gearprofile_y_coordinate(siggear_f_gearprofile_x[i], radius, theta);
    }

    /* using calc_f_gearprofile_vector() to calculate sigbase_f_gearprofile form siggear_f_gearprofile_x */
    Eigen::Vector3d siggear_f_gearprofile_vector = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigbase_f_gearprofile_vector = Eigen::Vector3d::Zero();

    /* test for each siggear_f_gearprofile_x */
    for (int i = 0; i < N; ++i) {
        siggear_f_gearprofile_vector << siggear_f_gearprofile_x[i],
                                        gear_design::siggear_f_gearprofile(siggear_f_gearprofile_x[i]),
                                        1.;
        sigbase_f_gearprofile_vector = \
            gear_design::calc_sigbase_f_gearprofile_vector(siggear_f_gearprofile_vector, sigbase_f_gearprofile_vector, radius, theta);

        /* test */
        ASSERT_NEAR(sigbase_f_gearprofile_vector[0], test_sigbase_f_gearprofile_x[i], 1e-6);
        ASSERT_NEAR(sigbase_f_gearprofile_vector[1], test_sigbase_f_gearprofile_y[i], 1e-6);
    }


    /* test */
    // for (int i = 0; i < N; ++i) {
    // }

    // ASSERT_NEAR((sig1_p[1] - sig1y_calculated_here) / sig1_p[1], 0., 1e-6) \
    //     << "r: " << r << ", theta: " << theta;
        // << ", sig0x_input: " << sig0x_input \
        // << ", sig0y_output: " << sig0y_output << std::endl;
}


int main(int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);

    for(int i = 0; i < 90000; ++i){
        RUN_ALL_TESTS();
    }

    return 0;
}



