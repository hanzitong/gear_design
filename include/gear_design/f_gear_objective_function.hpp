


#ifndef SIGBASE_F_GEAR_OBJECTIVE_FUNCTION_HPP
#define SIGBASE_F_GEAR_OBJECTIVE_FUNCTION_HPP

#include <vector>

namespace gear_design{

 

// double calc_sigbase_y_base_objfunc_shape(
//                                          const std::vector<double>& opt_vec,
//                                          std::vector<double>& grad,
//                                          void* f_data
//                                         );

double obj_calc_siggear_x_from_xbase(
                                     const std::vector<double>& opt_vec,
                                     std::vector<double>& grad,
                                     void* f_data
                                    );

double obj_calc_sigbase_y_from_xbase(
                                     const std::vector<double>& opt_vec,
                                     std::vector<double>& grad,
                                     void* f_data
                                    );




}   // namespace gear_design


#endif


