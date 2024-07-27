#pragma once
#include "controller/controller.hpp"
#include "units/units.hpp"
#include <cmath>

namespace controllers {
template <isQuantity In, isQuantity Out> struct pidConfig {
        double kP, kI = 0, kD = 0, iCut = 1;
        In iMax = In(0.0), inBase = In(1.0), maxThreshold = In(0.0);
        Out outBase = Out(1.0), outMax = Out(0.0);
};

template <isQuantity In, isQuantity Out> class PID : public Controller<In, In, Out> {
        pidConfig<In, Out>& config;
        double i = 0, perror = INFINITY;
        Out current;
    public:
        PID(pidConfig<In, Out>& config)
            : config(config) {}

      

        virtual Out update(const In &input) override { // todo: test this all
            double error = input.convert(config.inBase);
            if (perror == INFINITY) perror = error;
            double threshold = config.maxThreshold.convert(config.inBase);
            double max = config.outMax.convert(config.outBase);
            if ((threshold != 0) && (max != 0) && (error > threshold)) {
                current = units::copysign(config.outMax, input); // bang bang
            } else {
                if ((error < 0) != (perror < 0)) i *= config.iCut; // cut down I if the sign of the error changes
                else if ((config.iMax != In(0.0)) && (i >= config.iMax.convert(config.inBase)))
                    i = std::copysign(config.iMax.convert(config.inBase), i); // cap I
                else i += error; 
                double output = (error * config.kP) + (i * config.kI) + ((perror - error) * config.kD);
                perror = error;
                if (fabs(output) > max) output = std::copysign(max, error); // cap output
                current = output * config.outBase;
            }
            return current;
        }
        
        virtual void reset() {

        }
};
} // namespace jam