
#include <iostream>
#include <fstream>
#include <cmath>
#include <gear_design/quadratic_gear.hpp>



double gear(const int x, const int a) {
    return std::pow(x, 2.) * a;
}


int main() {
    std::ofstream dataFile;
    dataFile.open("data.dat");

    int a = 3;
    for (int x = -10; x <= 10; ++x) {
        double y = gear(x, a);
        dataFile << x << " " << y << "\n";
    }

    dataFile.close();
    return 0;
}


