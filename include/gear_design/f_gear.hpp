

#ifndef GEAR_DESIGN_F_GEAR_HPP
#define GEAR_DESIGN_F_GEAR_HPP

#include <Eigen/Core>

namespace gear_design {


struct GearParamFgear {
    double radius;  // mm
    double theta;   // rad
    double quad_a;  
    double quad_b;
    // double module;
    // double diameter;

    GearParamFgear(double r, double th, double a, double b):
                       radius(r), theta(th), quad_a(a), quad_b(b) {}
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



