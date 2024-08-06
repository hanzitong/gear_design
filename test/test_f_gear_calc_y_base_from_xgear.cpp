
#include <iostream>
#include <vector>
#include <random>
#include <cassert>
#include <gtest/gtest.h>

#include "../include/gear_design/f_gear.hpp"
#include "../include/gear_design/f_gear_objective_function.hpp"


TEST(SolutionTest, OnFunction) {
    /* set situation */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_r(20., 30.);
    double radius = dis_r(gen);
    std::uniform_real_distribution<double> dis_theta(0.1, M_PI - 0.1);
    double theta = dis_theta(gen);
    std::uniform_real_distribution<double> dis_xbase(-1., 1.);
    double x_base = dis_xbase(gen);

    /* prepare for calculation */
    gear_design::GearParamFgear params(radius, theta, x_base);

    /* calculate y_base with 2-ways */
    Eigen::Vector3d res_vec = Eigen::Vector3d::Zero();
    double y_base_1st = gear_design::calc_sigbase_y_gear_from_xgear(x_base, radius, theta, &res_vec);
    double x_gear = res_vec[1];
    double y_base_2nd = gear_design::calc_sigbase_f_gearprofile_y_coordinate(x_gear, radius, theta);

    /* assert y_base with 2-way calculation */
    ASSERT_NEAR(y_base_1st, y_base_2nd, 1e-6) << "here";




    /* calculate value of objective function */
    // std::vector<double> opt_vec = {x_base};
    // double y_obj = gear_design::obj_calc_sigbase_y_from_xgear_1variable(x_base, radius, theta);

}


int main(int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);

    for(int i = 0; i < 10000; ++i){
        RUN_ALL_TESTS();
    }

    return 0;
}

