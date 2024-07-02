

#ifndef GEAR_DESIGN_F_GEAR_HPP
#define GEAR_DESIGN_F_GEAR_HPP

namespace gear_design {


struct GearParamFgear {
    double radius;  // mm
    double theta;   // rad
    double quad_a;  
    double quad_b;
    // double module;
    // double diameter;

    GearParamFgear(double r, double th, double a, double b):
                       radius(r), theta(th), quad_a(a), quad_b(b) {}
};

// for cauculate x coordinate of f-gear which needs siggear_x_param for parameter.
double calc_gearshape_outline_fgear_x_coordinate(const double siggear_x_param, void* gearparam);
// for cauculate y coordinate of f-gear which needs siggear_y_param for parameter.
double calc_gearshape_outline_fgear_y_coordinate(const double siggear_x_param, void* gearparam);


}       // namespace gear_design


#endif  // GEAR_DESIGN_F_GEAR_HPP












