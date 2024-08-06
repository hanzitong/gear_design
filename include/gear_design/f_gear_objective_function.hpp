


#ifndef SIGBASE_F_GEAR_OBJECTIVE_FUNCTION_HPP
#define SIGBASE_F_GEAR_OBJECTIVE_FUNCTION_HPP

#include <vector>

namespace gear_design{

 
double obj_gearprofile1_gearprofile2_intersection_nlopt(
                                                        const std::vector<double>& x_base,
                                                        std::vector<double>& grad, 
                                                        void* f_data);
 
double calc_sigbase_y_base_objfunc_shape(
                                         const std::vector<double>& opt_vec,
                                         std::vector<double>& grad,
                                         void* f_data);

double obj_calc_sigbase_y_from_xgear_1variable(
                                               const std::vector<double>& opt_vec,
                                               std::vector<double>& grad,
                                               void* f_data);

// double calc_sigbase_y_base_objfunc_shape(
//                                          double& x_gear,    // variable to optmize 
//                                          double& x_base,    // input of sigbase_y_base
//                                          double& radius,    // parameter
//                                          double& theta);    // parameter




}   // namespace gear_design


#endif


