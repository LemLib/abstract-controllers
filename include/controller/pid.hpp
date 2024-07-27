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
        pidConfig<In, Out>& m_config;
        double m_integral = 0, m_prev_error = INFINITY;
        Out m_current;
    public:
        PID(pidConfig<In, Out>& config)
            : m_config(config) {}

      

        virtual Out update(const In &input) override { // todo: test this all
            double error = (this->target - input).convert(m_config.inBase);
            if (m_prev_error == INFINITY) m_prev_error = error;
            double threshold = m_config.maxThreshold.convert(m_config.inBase);
            double max = m_config.outMax.convert(m_config.outBase);
            if ((threshold != 0) && (max != 0) && (error > threshold)) {
                m_current = units::copysign(m_config.outMax, error); // bang bang
            } else {
                if ((error < 0) != (m_prev_error < 0)) m_integral *= m_config.iCut; // cut down I if the sign of the error changes
                else if ((m_config.iMax != In(0.0)) && (m_integral >= m_config.iMax.convert(m_config.inBase)))
                    m_integral = std::copysign(m_config.iMax.convert(m_config.inBase), m_integral); // cap I
                else m_integral += error; 
                double output = (error * m_config.kP) + (m_integral * m_config.kI) + ((m_prev_error - error) * m_config.kD);
                m_prev_error = error;
                if (fabs(output) > max) output = std::copysign(max, error); // cap output
                m_current = output * m_config.outBase;
            }
            return m_current;
        }

        virtual void reset() {

        }
};
} // namespace jam