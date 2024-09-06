


#ifndef SIGBASE_F_GEAR_OBJECTIVE_FUNCTION_HPP
#define SIGBASE_F_GEAR_OBJECTIVE_FUNCTION_HPP

#include <vector>

namespace gear_design{

// NAME SUGGESTION: GearParam&Coordinate
struct GearParamFgear {
    double radius;  // [m]
    double theta;   // [rad]
    double x_base;
    double x_gear;
    double y_base;
    double y_gear;  // temp

    GearParamFgear(double r, double th, double xbase):
                       radius(r), theta(th), x_base(xbase)
    {
    }
};


// NAME SUGGESTION: _obj_siggear_x_from_xbase
double obj_calc_siggear_x_from_xbase(
                                     const std::vector<double>& opt_vec,
                                     std::vector<double>& grad,
                                     void* f_data
                                    );

// double calc_sigbase_y_base_objfunc_shape(
//                                          const std::vector<double>& opt_vec,
//                                          std::vector<double>& grad,
//                                          void* f_data
//                                         );

// double obj_calc_sigbase_y_from_xbase(
//                                      const std::vector<double>& opt_vec,
//                                      std::vector<double>& grad,
//                                      void* f_data
//                                     );

struct GearParamFgear;


}   // namespace gear_design


#endif


