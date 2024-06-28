

#ifndef GEAR_DESIGN_QUADRATIC_GEAR_HPP
#define GEAR_DESIGN_QUADRATIC_GEAR_HPP

namespace gear_design {


struct GearParamQuadratic {
    double radius;  // mm
    double theta;   // rad
    double quad_a;  
    double quad_b;
    // double module;
    // double diameter;

    GearParamQuadratic(double r, double th, double a, double b):
                       radius(r), theta(th), quad_a(a), quad_b(b) {}
};

double calc_gearshape_outline_quadratic(const double x, void* gearparam);


// double calc_sig0y_quadratic(const double& x, const double& radius, const double& theta, const double& quad_a, const double& quad_b);
double calc_sig0y_quadratic(const double& x, void* gearparam);
// double calc_obj


}       // namespace gear_design
#endif  // GEAR_DESIGN_QUADRATIC_GEAR_HPP
