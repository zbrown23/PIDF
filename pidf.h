#pragma once

template <typename T> class PIDF {
private:
    // gains
    T k_p; // proportional
    T k_i; // integral
    T k_d; // differential
    T k_f; // feedforward
    /*
     * time constant of the low pass filter on the derivative term.
     * If tau is std::nullopt, then the filter is not enabled.
     */
    std::optional<T> tau;
    // clamping on the output.
    std::optional<T> out_min;
    std::optional<T> out_max;
    // integrator clamping
    std::optional<T> int_min;
    std::optional<T> int_max;
    // sample time (seconds)
    T sampleT;
    // state
    T integrator;
    T prev_error;
    T differentiator;
    T prev_measurement;
public:
    PIDF() : k_p(0), k_i(0), k_d(0), k_f(0), sampleT(0), integrator(0), prev_error(0), differentiator(0), prev_measurement(0) {}

    PIDF(T k_p, T k_i, T k_d, T k_f,
    T sampleT,
    std::optional<T> tau = std::nullopt,
    std::optional<T> out_min = std::nullopt,
    std::optional<T> out_max = std::nullopt,
    std::optional<T> int_min = std::nullopt,
    std::optional<T> int_max = std::nullopt) :
    k_p(k_p), k_i(k_i), k_d(k_d), k_f(k_f),
    sampleT(sampleT), tau(tau), out_min(out_min),
    out_max(out_max), int_min(int_min),
    int_max(int_max) {
        integrator = 0;
        prev_error = 0;
        differentiator = 0;
        prev_measurement = 0;
    }

    void setGains(T k_p, T k_i, T k_d, T k_f) {
        this->k_p = k_p;
        this->k_i = k_i;
        this->k_d = k_d;
        this->k_f = k_f;
    }

    void setKP(T k_p) {
        this->k_p = k_p;
    }

    void setKI(T k_i) {
        this->k_i = k_i;
    }

    void setKD(T k_d) {
        this->k_d = k_d;
    }

    void setKF(T k_f) {
        this->k_f = k_f;
    }

    T update(T setpoint, T measurement) {
        // calculate the error
        T error = setpoint - measurement;
        // calculate the proportional term
        T proportional = k_p * error;
        // calculate the integral term
        integrator = integrator + k_i * sampleT * (error + prev_error);
        // clamp the integrator
        if (int_max != std::nullopt && integrator > int_max) {
                integrator = int_max;
        }
        if (int_min != std::nullopt && integrator < int_min) {
                integrator = int_min;
        }
        // calculate derivative term in the low pass case
        if (tau != std::nullopt) {
            differentiator = -(2.0 * k_d * (measurement - prev_measurement)
                             + (2.0 * tau - sampleT) * differentiator)
                             / (2.0 * tau * sampleT);
        } else {
            // calculate the derivative term in the non-lowpass case.
            differentiator = k_d * (measurement - prev_measurement) / sampleT;
        }
        // calculate feedforward
        T feedforward = k_f * setpoint;
        // calculate the output value
        T output = proportional + integrator + differentiator + feedforward;
        // clamp the output
        if (out_max != std::nullopt && output > out_max) {
            output = out_max;
        }
        if (out_min != std::nullopt && output < out_min) {
            output = out_min;
        }
        return output;
    }
};