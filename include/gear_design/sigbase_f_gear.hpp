

#ifndef SIGBASE_F_GEAR_HPP
#define SIGBASE_F_GEAR_HPP

#include <vector>

namespace gear_design{

double obj_gearprofile1_gearprofile2_intersection(
                                                  const double& x_gear,     // variable (to be optimized)
                                                  const double& y_base,     // variable (to be optimized)
                                                  const double& x_base,     // input (known)
                                                  const double& radius,     // constant
                                                  const double& theta);     // constant
 
double obj_gearprofile1_gearprofile2_intersection_nlopt(
                                                        const double& x_base,
                                                        std::vector<double>& grad, 
                                                        void* f_data);
 

}   // namespace gear_design


#endif


