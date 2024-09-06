

#ifndef GEAR_DESIGN_F_GEAR_HPP
#define GEAR_DESIGN_F_GEAR_HPP

#include <Eigen/Core>

namespace gear_design {


double siggear_f_gearprofile(const double& x_gear);
// double siggear_real_gearprofile(const double& x_gear);       // define it in another file

// NAME SUGGESTION: calc_gearprofile_sigbase_y_gear_from_xbase
double calc_sigbase_y_gear_from_xbase(
    const double& x_base,
    const double& radius,
    const double& theta,
    Eigen::Vector3d* sigbase_vec = nullptr);

// PREVIOUS NAME: double calc_sigbase_f_gearprofile_x_coordinate(
double calc_gearprofile_sigbase_x_from_siggear_x(
    const double& siggear_x_param,
    const double& radius,
    const double& theta);

// PREVIOUS NAME: double calc_sigbase_f_gearprofile_y_coordinate(
double calc_gearprofile_sigbase_y_siggear_x(
    const double& siggear_f_gearprofile_x_param,
    const double& radius,
    const double& theta);

// NAME SUGGESTION: calc_gearprofile_siggear_Pvec_from_sigbase_Pvec
Eigen::Vector3d trans_siggear_to_sigbase(
        const Eigen::Vector3d& siggear_P,   // input
        Eigen::Vector3d& sigbase_P,         // output
        const double& radius,
        const double& theta
    );


}       // namespace gear_design

#endif  // GEAR_DESIGN_F_GEAR_HPP



