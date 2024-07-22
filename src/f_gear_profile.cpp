

#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>
#include <Eigen/Core>


namespace gear_design {


double siggear_f_gearprofile(const double& x) {
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
        return std::nan(" returned nan here ");
    }
}


double calc_sigbase_f_gearprofile_x_coordinate(const double& siggear_x_param, const double& radius, const double& theta) {
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
    const double& theta) {
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
Eigen::Vector3d calc_sigbase_f_gearprofile_vector(
    const Eigen::Vector3d& siggear_gearprofile,     // input (siggear_x, siggear_y, 1)
    Eigen::Vector3d& sigbase_gearprofile,           // output 
    const double& radius,               // cutter radius
    const double& theta) {              // the angle between axis-siggear_y_coordinate relative to axis-sigbase_x_coordinate

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



}   // namespace gear_design

