
#ifndef GEAR_DESIGN_QUADRATIC_GEAR_HPP
#define GEAR_DESIGN_QUADRATIC_GEAR_HPP


namespace gear_design {

struct GearParam {
    double module;
    double diameter;
};


double calc_gearshape_outline_quadratic(const double x, void* gearparam);


}       // namespace gear_design

#endif      // GEAR_DESIGN_QUADRATIC_GEAR_HPP
