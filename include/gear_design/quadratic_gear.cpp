
#include <iostream>
#include <vector>


// bool calc_gear_right_shape_quadratic(const std::vector<double>& input, std::vector<double> output, const int length) {
bool calc_gear_right_shape_quadratic(const std::vector<double>& input, std::vector<double> output) {
    // defence
    if (input.size() != output.size()) {
        throw std::invalid_argument("Input and Output array size is not same.");
    }

    // y = - x^2    
    for (int i = 0; i < input.size(); ++i) {
        output[i] = -1. * input[i] * input[i];
    }

    return true;
}


bool calc_gear_left_shape_quadratic(const std::vector<double>& input, std::vector<double>& output) {
    calc_gear_right_shape_quadratic(input, output);
    // calc axis standing for the center of its range.
    // int index = std::min(output);
    // int index = std::min(output.begin(), output.end());
    // std::cout << index << std::endl;
    int size = input.size();
    for (int i = 0; i < size; ++i) {
        std::swap(output[i], output[size - i]);     // inversion about y-axis
    }


    return true;
}


bool calc_gear_top_shape_circle(const std::vector<double>& input, std::vector<double>& output) {  // 3rd argument: length???

    return true;
}

