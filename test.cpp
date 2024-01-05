#include "pidf.h"
#include "matplot/matplot.h"
#include <cmath>

constexpr int sample_rate = 1000;
constexpr double sample_t = 1 / static_cast<double >(sample_rate);

class FirstOrderPlant {
public:
    FirstOrderPlant(double timeConstant)
        : tau(timeConstant), y(0.0) {}

    double update(double input, double deltaTime) {
        double dy_dt = (input - y) / tau;
        y += dy_dt * deltaTime;
        return y;
    }
private:
    double tau; // Time constant
    double y;   // System output
};

PIDF<double > pidf = PIDF<double>(1, 0.5, 0.0, 0.0, sample_t);
FirstOrderPlant plant = FirstOrderPlant(0.01);

int main() {
using namespace matplot;
    std::vector<double> t = linspace(0, 2*pi, 2.0 * pi / sample_t);
    std::vector<double> setpoint = transform(t, [](auto x) { return sin(x); });
    std::vector<double> output(t.size());
    for (int i = 0; i < t.size(); ++i) {
        if(i == 0) {
            output[i] = plant.update(pidf.update(setpoint[i], 0), sample_t);
        } else {
            output[i] = plant.update(pidf.update(setpoint[i], output[i-1]), sample_t);
        }
    }
    plot(t, output);
    hold(on);
    plot(t, setpoint);
    show();
    return 0;
}