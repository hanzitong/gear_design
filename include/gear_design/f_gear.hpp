

#ifndef GEAR_DESIGN_F_GEAR_HPP
#define GEAR_DESIGN_F_GEAR_HPP

#include <Eigen/Core>

namespace gear_design {


struct GearParamFgear {
    double radius;  // mm
    double theta;   // rad
    double x_base;  // input for objfunction of 

    // double quad_a;      // maybe, no need
    // double quad_b;      // maybe, no need

    GearParamFgear(double r, double th, double xbase):
                       radius(r), theta(th), x_base(xbase) {}
};


double siggear_f_gearprofile(const double& x);
double calc_sigbase_f_gearprofile_x_coordinate(const double& siggear_x_param, const double& radius, const double& theta);
double calc_sigbase_f_gearprofile_y_coordinate(
    const double& siggear_f_gearprofile_x_param,
    const double& radius,
    const double& theta);
 
// double calc_sigbase_f_gearprofile_x_coordinate(const double& , const double& , const double&);
// double calc_sigbase_f_gearprofile_y_coordinate(const double& , const double& , const double&);

Eigen::Vector3d
calc_sigbase_f_gearprofile_vector(const Eigen::Vector3d& siggear_P, Eigen::Vector3d& sigbase_P, const double& radius, const double& theta);

}       // namespace gear_design


#endif  // GEAR_DESIGN_F_GEAR_HPP



