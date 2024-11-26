
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
// double quad_calc_siggear_yprofile_from_siggear_x(const double& siggear_x);
// double real_calc_siggear_yprofile_from_sigger_x(const double& siggear_x);   // define it in future




/* Implementation */
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


}   // namespace gear_design


#endif


