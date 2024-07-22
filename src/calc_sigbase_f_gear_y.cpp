
#include <iostrem>
#include <vector>
#include <nlopt.hpp>

#include "../../include/gear_design/f_gear.hpp"


// double obj_gearprofile1_gearprofile2_intersection(const double& x, void* chank) {
//     GearParamQuadratic* params = static_cast<GearParamQuadratic>(chank);
//     const double radius = params -> radius;
//     const double theta = params -> theta;
    
double obj_gearprofile1_gearprofile2_intersection(
                                                  const double& x_gear,     // variable (to be optimized)
                                                  const double& y_base,     // variable (to be optimized)
                                                  const double& x_base,     // input (known)
                                                  const double& radius,     // constant
                                                  const double& theta) {    // constant
    double cos = std::cos(theta);
    double sin = std::sin(theta);
    double y_base = siggear_f_gearprofile(x_gear);
    double res = x_base * (cos + sin) - y_base * (sin + cos) - radius * sin + radius * cos - x_base - y_base;


    return res;
}

double obj_gearprofile1_gearprofile2_intersection_nlopt(
                                                        const double& x_base,
                                                        std::vector<double>& grad, 
                                                        void* f_data) {
    gear_design::GearParamFgear* params = static_cast<gear_design::GearParamFgear*>(f_data);
    double radius = params -> radius;
    double theta = params -> theta;

    double cos = std::cos(theta);
    double sin = std::sin(theta);
    double y_base = siggear_f_gearprofile(x_gear);
    double res = x_base * (cos + sin) - y_base * (sin + cos) - radius * sin + radius * cos - x_base - y_base;


    return res;
}



double calc_gearprofile1_gearprofile2_intersection(double x_base, double radius, double theta) {
    nlopt::opt obj_intersection(nlopt::LN_COBYLA, 2);

    obj_intersection.set_min_objective(ofj_gearprofile1_gearprofile2_intersection_nlopt);



    return -1;
}

