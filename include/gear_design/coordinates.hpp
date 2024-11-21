

#ifndef GEAR_DESIGN_COORDINATES_HPP
#define GEAR_DESIGN_COORDINATES_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>
#include <Eigen/Core>
#include <nlopt.hpp>
#include "gear_design/objfunctions.hpp"     // Be careful of circular reference !!

namespace gear_design {

/* PROTOTYPE DECRALATION */
double _calc_gearprofile_sigbase_x_from_siggear_x(
        const double& siggear_x_param,
        const double& radius,
        const double& theta
    );

double _calc_gearprofile_sigbase_y_siggear_x(
        const double& siggear_f_gearprofile_x_param,
        const double& radius,
        const double& theta
    );

Eigen::Vector3d trans_Pvec_from_siggear_to_sigbase(
        const Eigen::Vector3d& siggear_P,   // input
        Eigen::Vector3d& sigbase_P,         // output
        const double& radius,
        const double& theta
    );

// Eigen::Vector3d trans_Pvec_from_siggear_to_silgbase(void);

// USED FOR INTEGRAL
// NAME SUGGESTION: calc_gearprofile_sigbase_y_gear_from_xbase
double calc_sigbase_y_gear_from_xbase(
    const double& x_base,
    const double& radius,
    const double& theta,
    Eigen::Vector3d* sigbase_vec = nullptr);


/* IMPLEMENTATION */
double _calc_gearprofile_sigbase_x_from_siggear_x(
    const double& siggear_x_param,
    const double& radius,
    const double& phi)
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
    In this figure, phi is minus.
    phi = theta - M_PI / 2
    
    */

    double cos = std::cos(phi);
    double sin = std::sin(phi);

    // transform along siggear_y-axis and rotate
    return siggear_x_param * cos - (radius + quad_calc_siggear_yprofile_from_siggear_x(siggear_x_param)) * sin;
}


double _calc_gearprofile_sigbase_y_from_siggear_x(
    const double& siggear_f_gearprofile_x_param,
    const double& radius,
    const double& phi)
{
    /*
    for cauculate y coordinate of f-gear which needs siggear_y_param as a parameter.

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
    In this fugure, phi is minus.
    phi = theta - M_PI / 2

    */

    const double cos = std::cos(phi);
    const double sin = std::sin(phi);

    // transform along siggear_y-axis and then rotate
    return siggear_f_gearprofile_x_param * sin + (radius + quad_calc_siggear_yprofile_from_siggear_x(siggear_f_gearprofile_x_param)) * cos;
}


Eigen::Vector3d trans_Pvec_from_siggear_to_sigbase(
    const Eigen::Vector3d& siggear_gearprofile,     // input (siggear_x, siggear_y, 1)
    Eigen::Vector3d& sigbase_gearprofile,           // output 
    const double& radius,     // cutter radius
    // const double& theta)      // the angle between axis-siggear_y_coordinate relative to axis-sigbase_x_coordinate
    const double& phi)
{
    /*
    This function conducts only transformation.
    This function transforms vectors from siggear to sigbase.
    Counter-clockwise is its default rotation direction.
    If phi > 0, It transforms counter-clockwise.

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
    
    phi: the angle between axis-siggear_y_coordinate relative to axis-sigbase_y_coordinate,
            with the axis-sigbase_y_coordinate as the reference.                               
         In this figure, phi is minus. 
         phi = theta - M_PI / 2

    using homogeneous transformation matrix, parallel move and then, apply rotation
                        (cos(phi),    -sin(phi),    0)   (1    0   0)
    sibbase_T_siggear = (sin(phi),    cos(phi),     0) * (0    1   r)
                        (0,           0,            1)   (0    0   1)
 
    Its inverse matrix, sigbase_T_siggear, is like below.
                        (cos(phi),    sin(phi),     0)
    siggear_T_sigbase = (-sin(phi),   cos(phi),    -r)
                        (0,           0,            1)
   */

    // const double phi = theta - M_PI / 2.;
    const double cos = std::cos(phi);
    const double sin = std::sin(phi);

    Eigen::Matrix3d sigbase_T_siggear = Eigen::Matrix3d::Zero();
    sigbase_T_siggear << cos,       -1. * sin,  -1. * radius * sin,
                         sin,       cos,        radius * cos,
                         0.,        0.,         1.;
    
    sigbase_gearprofile = sigbase_T_siggear * siggear_gearprofile;
    return sigbase_gearprofile;
}

Eigen::Vector3d trans_Pvec_from_sigbase_to_siggear(
    const Eigen::Vector3d& sigbase_gearprofile,
    Eigen::Vector3d& siggear_gearprofile,
    const double& radius,
    // const double& theta)
    const double& phi)
{
    /*
    inverse matrix of sigbase_T_siggear: 

    Eigen::Matrix3d siggear_T_sigbase = Eigen::Matrix3d::Zero();
        siggear_T_sigbase << cos,       sin,        0.,
                             -1. * sin, cos,        -1. * radius,
                             0.,        0.,         1.;
    */

    // const double phi = theta - M_PI / 2.;
    const double cos = std::cos(phi);
    const double sin = std::sin(phi);
 
    Eigen::Matrix3d siggear_T_sigbase = Eigen::Matrix3d::Zero();
    /* In this case, clockwise is default direction */
    /* In other word, if phi > 0, it transforms to clockwise direction */
    siggear_T_sigbase << cos,        sin,    0.,
                         -1. * sin,  cos,    -1. * radius,
                         0.,         0.,     1.;

    /* normally, it should be this form considering direction of transformation */
    // siggear_T_sigbase << cos,   -1. * sin,  0.,
    //                      sin,   cos,        -1. * radius,
    //                      0.,    0.,         1.;

    siggear_gearprofile = siggear_T_sigbase * sigbase_gearprofile;
    
    return siggear_gearprofile;
}


// NAME SUGGESTION: calc_gearprofile
// PREVOUS NAME: double calc_sigbase_y_gear_from_xbase(
// double calc_sigbase_y_gear_from_xbase(
double calc_gearprofile_sigbase_y_from_sigbase_x(
    const double& x_base,
    const double& radius,
    const double& theta,
    Eigen::Vector3d* sigbase_vec)   // optional argument (for test)
{
    /*
    STEP1: numerically calculate x_gear from equation1 below.
    STEP2: calculate y_base from 2 ways.
            WAY1: use equation2 below.
            WAY2: transform from siggear to sigbase.

    Vec: pointing a coordinate on f_gear_profile.
    R:   homegenous transformation matrix.

    0 = sigbase_Vec - siggear_Vec
    0 = sigbase_Vec - sigbase_R_siggear * siggear_Vec

    (0.)   (x_base)   (cos(theta), -sin(theta), -r * sin(theta))   (x_gear)
    (0.) = (y_base) - (sin(theta), cos(theta),  r * cos(theta) ) * (y_gear)
    (0.)   (1.    )   (0.,         0.,          1.             )   (1.    )

    (0.)   (x_base)   (x_base - x_gear * cos + y_gear * sin + r * sin)
    (0.) = (y_base) - (y_base - x_gear * sin - y_gear * cos - r * cos)
    (0.)   (1.    )   (1.                                            )

    0. = x_base - x_gear * cos + y_gear * sin + r * sin      (equation 1)
    0. = y_base - x_gear * sin - y_gear * cos - r * cos      (equation 2)
    */

    // STEP1: calc Xgear from Xbase using equation1
    /* prepare optimization materials */
    std::vector<double> opt_variable(1, 50.);
    double val_objfunc = 0.;
    GearParamFgear param(radius, theta, x_base);
    nlopt::opt calc_y_base = nlopt::opt(nlopt::LN_COBYLA, 1);
    calc_y_base.set_min_objective(obj_calc_siggear_x_from_xbase, (void*)&param);
    calc_y_base.set_stopval(1e-6);

    /* To set lower or upper bounds, check the value of theta */
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

    /* calculate x_gear */
    calc_y_base.optimize(opt_variable, val_objfunc);
    assert(std::abs(val_objfunc) <= 1e-6);      // nearly equal
    assert(false);
    assert(true);
    double x_gear = opt_variable[0];
    

    // STEP2
    /* cache */
    double sin = std::sin(theta);
    double cos = std::cos(theta);

    /* WAY1: output y_base (use equation 2 in paper) */
    double y_base = \
        x_gear * sin - gear_design::quad_calc_siggear_yprofile_from_siggear_x(x_gear) * cos - radius * cos;
    /* WAY2: output y_base (use trans_sigbase_y_gear_from_xbase()) */
    // TODO: to implement WAY2



    /* for test case */
    if (sigbase_vec) {      // if sigbase_vec is given
        (*sigbase_vec)[0] = x_base;
        (*sigbase_vec)[1] = y_base;
        (*sigbase_vec)[2] = 1.;
    }


    return y_base;
}


}       // namespace gear_design

#endif  // GEAR_DESIGN_COORDINATES_HPP


