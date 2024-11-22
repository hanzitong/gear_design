
#ifndef GEAR_DESIGN_OBJFUNCTIONS_HPP
#define GEAR_DESIGN_OBJFUNCTIONS_HPP

#include <vector>
#include <nlopt.hpp>
#include "gear_design/coordinates.hpp"  // Be careful of circular reference !!


namespace gear_design{

struct ConditionParam {
    double radius;  // [m]
    double phi;   // [rad]
    double x_base;
    double x_gear;
    double y_base;
    double y_gear;  // temp

    ConditionParam(double r, double phi, double xbase):
                       radius(r), phi(phi), x_base(xbase)
    {
    }
};

/* Prototype Declaration */
double quad_calc_siggear_yprofile_from_siggear_x(const double& siggear_x);
// double real_calc_siggear_yprofile_from_sigger_x(const double& siggear_x);   // define it in future

// NAME SUGGESTION: _obj_siggear_x_from_xbase
double obj_calc_siggear_x_from_xbase(
                                     const std::vector<double>& opt_vec,
                                     std::vector<double>& grad,
                                     void* f_data
                                    );

/* Implementation */
// PREVIOUS NAME: siggear_f_gearprofile(const double& x)
double quad_calc_siggear_yprofile_from_siggear_x(const double& siggear_x)
{
    /*
    Define gear profile as the function format here.
    This is a EXAMPLE gear profile

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

    if (siggear_x <= -1.) {     // left flank
        return a * siggear_x + 32.;
    } else if (1. <= siggear_x) {   // right flank
        return -1. * a * siggear_x + 32.;
    } else if (-1. <= siggear_x && siggear_x <= 1.) {   // tooth tip
        return -4. * siggear_x * siggear_x + 20.;
    } else {
        return std::nan("quad_gear invalid");
    }
}


double _obj_calc_siggear_x_from_sigbase_x(
                                     const std::vector<double>& opt_vec,
                                     std::vector<double>& grad,
                                     void* f_data
                                    )
{
    /* cast parameters from input structs (nlopt manner) */
    gear_design::ConditionParam* params = static_cast<gear_design::ConditionParam*>(f_data);

    std::cout << "here !!!!!!!!!!!!!!" << std::endl;

    /* rename & cache */
    double radius = params -> radius;
    double phi = params -> phi;
    double cos = std::cos(phi);
    double sin = std::sin(phi);
    double x_base = params -> x_base;   // input
    double x_gear = opt_vec[0];         // variable to optimize
    double y_gear = quad_calc_siggear_yprofile_from_siggear_x(x_gear);
    // double y_gear = gear_design::siggear_f_gearprofile(x_gear);

    /* objective function for calculating x_gear */
    double res = x_base - x_gear * cos + y_gear * sin + radius * sin;   // equaton 1 in paper


    // return res;         // for using root finding algorithm
    return res * res;   // for using optimization algorithm
}


}   // namespace gear_design


#endif


