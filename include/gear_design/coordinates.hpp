

#ifndef GEAR_DESIGN_COORDINATES_HPP
#define GEAR_DESIGN_COORDINATES_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <nlopt.hpp>

#include <cassert>

#include "gear_design/objfunctions.hpp"     // Be careful of circular reference !!

namespace gear_design {

/* ===== PROTOTYPE DECRALATION ===== */
double _calc_gearprofile_sigbase_x_from_siggear_x(
        const double& siggear_x_param,
        const double& radius,
        const double& phi
    );

double _calc_gearprofile_sigbase_y_siggear_x(
        const double& siggear_f_gearprofile_x_param,
        const double& radius,
        const double& phi
    );

Eigen::Vector3d trans_Pvec_from_siggear_to_sigbase(
        const Eigen::Vector3d& siggear_P,   // input
        Eigen::Vector3d& sigbase_P,         // output
        const double& radius,
        const double& phi
    );

Eigen::Vector3d trans_Pvec_from_sigbase_to_siggear(
        const Eigen::Vector3d& sigbase_gearprofile, // input
        Eigen::Vector3d& siggear_gearprofile,       // output
        const double& radius,
        const double& phi
    );

double calc_sigbase_yprofile_from_sigbase_x(
        const double& x_base,
        const double& radius,
        const double& phi,
        Eigen::Vector3d* sigbase_vec = nullptr  // for test
    );
/* objective function of calc_sigbase_yprofile_from_sigbase_x */
double _obj_calc_siggear_x_from_sigbase_x(
                                     const std::vector<double>& opt_vec,
                                     std::vector<double>& grad,
                                     void* f_data
                                    );


/* ====== IMPLEMENTATION ===== */
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
    theta: the angle between axis-siggear_y_coordinate relative to axis-sigbase_x_coordinate
    
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


// not finish  2024/11/21
double calc_sigbase_yprofile_from_sigbase_x(
    const double& sigbase_x,
    const double& radius,
    const double& phi,
    Eigen::Vector3d* sigbase_Pvec)   // optional argument (for test)
{
    /*
    1. solve equation 1 numerically. x_base is known. x_gear is variable.
    2. solve equation 2 explicitly. x_gear is known. y_base is variable.

    Vec: pointing a coordinate on f_gear_profile.
    T:   homegenous transformation matrix.

    0 = sigbase_Vec - siggear_Vec
    0 = sigbase_Vec - sigbase_T_siggear * siggear_Vec

    (0.)   (x_base)   (cos(theta), -sin(theta), -r * sin(theta))   (x_gear)
    (0.) = (y_base) - (sin(theta), cos(theta),  r * cos(theta) ) * (y_gear)
    (0.)   (1.    )   (0.,         0.,          1.             )   (1.    )

    (0.)   (x_base)   (x_base - x_gear * cos + y_gear * sin + r * sin)
    (0.) = (y_base) - (y_base - x_gear * sin - y_gear * cos - r * cos)
    (0.)   (1.    )   (1.                                            )

    0. = x_base - x_gear * cos + y_gear * sin + r * sin      (equation 1)
    0. = y_base - x_gear * sin - y_gear * cos - r * cos      (equation 2)

    */

    /* nlopt materials */
    nlopt::opt optimizer_siggear_x = nlopt::opt(nlopt::LN_COBYLA, 1); // (algorithm, variable_num)
    double output_objfunc = 0.;
    double init_value = std::nan("init_value is not set");
    gear_design::ConditionParam condition(radius, phi, sigbase_x);
    optimizer_siggear_x.set_min_objective(_obj_calc_siggear_x_from_sigbase_x, (void*)&condition);
    optimizer_siggear_x.set_stopval(1e-6);
    if (phi == 0.) {    // if no rotation
        double siggear_yprofile_phi0vertex = \
            gear_design::quad_calc_siggear_yprofile_from_siggear_x(sigbase_x);
        return siggear_yprofile_phi0vertex + radius; // equal to sigbase_yprofile
    } else if (0. < phi && phi < M_PI / 2.) {
        std::vector<double> lb_siggear_x(1, 0.);
        optimizer_siggear_x.set_lower_bounds(lb_siggear_x);
        // init_value = 0.01;
        init_value = 0.;
    } else if (-1. * M_PI / 2. < phi && phi < 0.) {
        std::vector<double> ub_siggear_x(1, 0.);
        optimizer_siggear_x.set_upper_bounds(ub_siggear_x);
        // init_value = -0.01;
        init_value = 0.;
    } else {
        std::cerr << "===== no condition =====" << std::endl;
        return std::nan("calc_sigbase_yprofile_from_sigbase_x");
    }

    // std::cerr << "here here!!!!!!!!!!!!!!";
    // std::cerr << init_value;

    /* calculate(optimize) x_gear with nlopt */
    if (init_value == std::nan("")) std::cerr << "here is nan" << std::endl;
    std::vector<double> optimization_variable(1, init_value);
    optimizer_siggear_x.optimize(optimization_variable, output_objfunc);
    double optimal_siggear_x = optimization_variable[0];

    std::cerr << " ============" << optimal_siggear_x;

    /* calculate sigbase_y by substituting optimal_siggear_x to equation 2 */
    double sin = std::sin(phi);
    double cos = std::cos(phi);
    /* equation 2 in paper */
    double optimal_sigbase_y = \
        optimal_siggear_x * sin \
        + gear_design::quad_calc_siggear_yprofile_from_siggear_x(optimal_siggear_x) \
        * cos + radius * cos;

    /* for test case */
    if (sigbase_Pvec) {      // If sigbase_vec is given, it means ture.
        (*sigbase_Pvec)[0] = sigbase_x;
        (*sigbase_Pvec)[1] = optimal_sigbase_y;
        // (*sigbase_Pvec)[2] = 1.; // for position vector
        (*sigbase_Pvec)[2] = optimal_siggear_x; // for test case
    }


    return optimal_sigbase_y;
}

/* objective function of calc_sigbase_yprofile_from_sigbase_x */
double _obj_calc_siggear_x_from_sigbase_x(
                                     const std::vector<double>& opt_vec,
                                     std::vector<double>& grad,
                                     void* f_data
                                    )
{
    /* cast parameters from input structs (nlopt manner) */
    gear_design::ConditionParam* params = static_cast<gear_design::ConditionParam*>(f_data);

    /* rename & cache */
    double radius = params -> radius;
    double phi = params -> phi;
    double cos = std::cos(phi);
    double sin = std::sin(phi);
    double sigbase_x = params -> x_base;   // input
    double siggear_x = opt_vec[0];         // variable to optimize

    double siggear_y = quad_calc_siggear_yprofile_from_siggear_x(siggear_x);
    /* objective function for calculating x_gear */
    double res = sigbase_x - siggear_x * cos + siggear_y * sin + radius * sin;   // equaton 1 in paper


    // return res;         // for using root finding algorithm
    return res * res;   // for using optimization algorithm
}


}       // namespace gear_design

#endif  // GEAR_DESIGN_COORDINATES_HPP

