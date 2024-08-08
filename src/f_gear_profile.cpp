
#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>
#include <Eigen/Core>
#include <nlopt.hpp>

#include "../include/gear_design/f_gear.hpp"
#include "../include/gear_design/f_gear_objective_function.hpp"


namespace gear_design {


double siggear_f_gearprofile(const double& x)
{
    /*
    example gear profile

                   ^ siggear_y_coordinate
                   |
                .-----.-------------------
             .'    |    '.            ^
            /.     |     .\           |
           / .     |     . \          |
          /  .     |     .  \         | 32
         /   .     |     .   \        |
        /    .     |     .    \       v
     --/-----.-----.-----.-----\----------> siggear_x_coordinate
      /      .    0|     .      \
     /       .     |     .       \
       left  .  tooth    . right
       flank .   tip     . flank
            x=-1        x=1
    */    

    const double a = 16.;         // example value

    if (x <= -1.) {     // left flank
        return a * x + 32.;
    } else if (1. <= x) {   // right flank
        return -1. * a * x + 32.;
    } else if (-1. <= x && x <= 1.) {   // tooth tip
        return -4. * x * x + 20.;
    } else {
        return std::nan("siggear_f_gearprofile()");
    }
}


double calc_sigbase_f_gearprofile_x_coordinate(
    const double& siggear_x_param,
    const double& radius,
    const double& theta)
{
    /*
    for cauculate x coordinate of f-gear which needs siggear_x_param for parameter.

      sigbase_y_coordinate ^
                           |         ^ siggear_y_coordinate
                    _______| `.     /
                   ^       |    `. /  
                   |       |      /`.
            radius |       | phi /    `.      phi: its minus case
                   |       |""\ /        `.
                   |       |  v/           '. 
                   |       |  /             ^'> siggear_x_coordinate
                   |       | /<.           /  
                   v       |/   \theta_deg/ 
    -----------------------.-------------/-------> sigbase_x_coordinate
                         0 |`.          / radius
                           |   `.      / 
                           |      `.  v
                           |         '.
    
    */

    double phi = theta - M_PI / 2.;
    double cos = std::cos(phi);
    double sin = std::sin(phi);

    // transform along siggear_y-axis and rotate
    return siggear_x_param * cos - (radius + siggear_f_gearprofile(siggear_x_param)) * sin;
}


double calc_sigbase_f_gearprofile_y_coordinate(
    const double& siggear_f_gearprofile_x_param,
    const double& radius,
    const double& theta)
{
    /*
    for cauculate y coordinate of f-gear which needs siggear_y_param for parameter.

      sigbase_y_coordinate ^
                           |         ^ siggear_y_coordinate
                    _______| `.     /
                   ^       |    `. /  
                   |       |      /`.
            radius |       | phi /    `.       phi: its minus case
                   |       |""\ /        `.
                   |       |  v/           '. 
                   |       |  /             ^'> siggear_x_coordinate
                   |       | /<.           /  
                   v       |/   \theta_deg/ 
    -----------------------.-------------/-------> sigbase_x_coordinate
                         0 |`.          / radius
                           |   `.      / 
                           |      `.  v
                           |         '.
    
    */

    const double phi = theta - M_PI / 2.;
    const double cos = std::cos(phi);
    const double sin = std::sin(phi);

    // transform along siggear_y-axis and then rotate
    return siggear_f_gearprofile_x_param * sin + (radius + siggear_f_gearprofile(siggear_f_gearprofile_x_param)) * cos;
}


/* actually, it conducts only transformation */
Eigen::Vector3d trans_siggear_to_sigbase(
    const Eigen::Vector3d& siggear_gearprofile,     // input (siggear_x, siggear_y, 1)
    Eigen::Vector3d& sigbase_gearprofile,           // output 
    const double& radius,     // cutter radius
    const double& theta)      // the angle between axis-siggear_y_coordinate relative to axis-sigbase_x_coordinate
{
    /*
    this function transforms siggear_gearprofile to sigbase_gearprofile.
    
      sigbase_y_coordinate ^
                           |         ^ siggear_y_coordinate
                  ---------|  .     /
                   ^       |    `. /  
                   |       |     0*`.
            radius |       | phi /    `> siggear_x_coordinate
                   |       |""\ /       `.     
                   |       |  v/          '.   phi: its minus case
                   |       |  /             '.
                   |       | /<.theta_deg  ^  
                   v       |/   \         / 
    -----------------------*-------------/-------> sigbase_x_coordinate
                         0 |`.          / radius
                           |   `.      / 
                           |      `.  v
                           |         '.
    
    */

    /*
    using homogeneous transformation matrix, parallel move and then, apply rotation

                        (cos(theta),    -sin(theta),    0)   (1    0   0)
    sibbase_T_siggear = (sin(theta),    cos(theta),     0) * (0    1   r)
                        (0,             0,              1)   (0    0   1)
    */

    /* 
    phi: the angle between axis-siggear_y_coordinate relative to axis-sigbase_y_coordinate,
            with the axis-sigbase_y_coordinate as the reference.                               
    */
    const double phi = theta - M_PI / 2.;

    /* cache */
    const double cos = std::cos(phi);
    const double sin = std::sin(phi);

    Eigen::Matrix3d sigbase_T_siggear = Eigen::Matrix3d::Zero();
    sigbase_T_siggear << cos,       -1. * sin,  -1. * radius * sin,
                         sin,       cos,        radius * cos,
                         0.,        0.,         1.;
    
    sigbase_gearprofile = sigbase_T_siggear * siggear_gearprofile;

    /* memo: inverse matrix of sigbase_T_siggear */
    // Eigen::Matrix3d siggear_T_sigbase = Eigen::Matrix3d::Zero();
    //     siggear_T_sigbase << cos,       sin,        0.,
    //                          -1. * sin, cos,        -1. * radius,
    //                          0.,        0.,         1.;

    return sigbase_gearprofile;
}


double calc_sigbase_y_gear_from_xbase(
    const double& x_base,
    const double& radius,
    const double& theta,
    Eigen::Vector3d* sigbase_vec)   // optional argument (for test)
{
    /* prepare optimization materials */
    std::vector<double> opt_variable(1, 50.);
    double val_objfunc = 0.;
    GearParamFgear param(radius, theta, x_base);
    nlopt::opt calc_y_base = nlopt::opt(nlopt::LN_COBYLA, 1);
    calc_y_base.set_min_objective(obj_calc_siggear_x_from_xbase, (void*)&param);
    calc_y_base.set_stopval(1e-6);

    /* check the value of theta */
    if (0. < theta && theta < M_PI / 2.) {
        // std::vector<double> ub(1, 0.);
        // calc_y_base.set_upper_bounds(ub);
        // calc_y_base.set_upper_bounds(std::vector<double>(1, radius));   // correct ???????. is't in another function???
    } else if (M_PI / 2. < theta && theta < M_PI) {
        // std::vector<double> lb(1, 0.);
        // calc_y_base.set_lower_bounds(lb);
        // calc_y_base.set_lower_bounds(std::vector<double>(1, 0.));
    } else if (theta == M_PI / 2.) {
        // do nothing
    } else {
        return std::nan("in calc_sigbase_y_gear_from_xgear(), out of theta range");
    }

    /* calculate y_base */
    calc_y_base.optimize(opt_variable, val_objfunc);
    assert(std::abs(val_objfunc) <= 1e-6);      // nearly equal
    assert(false);
    assert(true);

    /* rename and cache */
    double x_gear = opt_variable[0];
    double sin = std::sin(theta);
    double cos = std::cos(theta);

    /* output y_base (use equation 2 in paper) */
    double y_base = \
        x_gear * sin - gear_design::siggear_f_gearprofile(x_gear) * cos - radius * cos;

    /* for test case */
    if (sigbase_vec) {      // if sigbase_vec is given
        (*sigbase_vec)[0] = x_base;
        (*sigbase_vec)[1] = y_base;
        (*sigbase_vec)[2] = 1.;
    }


    return y_base;
}



}   // namespace gear_design

