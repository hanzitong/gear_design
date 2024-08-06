

#ifndef GEAR_DESIGN_F_GEAR_HPP
#define GEAR_DESIGN_F_GEAR_HPP

#include <Eigen/Core>

namespace gear_design {


struct GearParamFgear {
    double radius;  // [m]
    double theta;   // [rad]
    double x_base;  // input for calc_sigbase_y_base()

    GearParamFgear(double r, double th, double xbase):
                       radius(r), theta(th), x_base(xbase)
    {
    }
};


double siggear_f_gearprofile(const double& x_gear);
// double siggear_real_gearprofile(const double& x_gear);       // define it in another file

double calc_sigbase_y_gear_from_xgear(
    const double& x_base,
    const double& radius,
    const double& theta,
    Eigen::Vector3d* sigbase_vec = nullptr);


double calc_sigbase_f_gearprofile_x_coordinate(
    const double& siggear_x_param,
    const double& radius,
    const double& theta);
double calc_sigbase_f_gearprofile_y_coordinate(
    const double& siggear_f_gearprofile_x_param,
    const double& radius,
    const double& theta);

Eigen::Vector3d calc_sigbase_f_gearprofile_vector(
    const Eigen::Vector3d& siggear_P,
    Eigen::Vector3d& sigbase_P,
    const double& radius,
    const double& theta);




}       // namespace gear_design

#endif  // GEAR_DESIGN_F_GEAR_HPP



