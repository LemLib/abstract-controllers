#pragma once
#include "controller/controller.hpp"
#include "units/units.hpp"
#include <cmath>

namespace controllers {
template <isQuantity In, isQuantity Out> struct pidConfig {
        double kP, kI = 0, kD = 0; // Constants for Proportional, Integral, Derivative

        /**
         * @brief Integral is multiplied by this amount if sign of error changes (you
         * can safely ignore if kI is 0)
         */
        double integralCut = 1;

        /**
         * @brief The maximum absolute value of the integral.
         */
        In integralMax = In(0.0);

        /**
         * @brief The highest absolute value that can be outputted.
         */
        Out outMax = Out(0.0);

        /**
         * @brief If the absolute value of the error is above this number, use
         * bang-bang instead of PID. Set to NaN to disable.
         */
        In bbangThreshold = In(0.0);

        /**
         * @brief The value to output when bang-bang is active, multiplied by the sign
         * of the error.
         */
        Out bbangValue = Out(0.0);

        /**
         * @brief The base unit to use for the input (effectively scales constants)
         */
        In inBase = In(1.0);

        /**
         * @brief The base unit to use for the output (effectively scales constants)
         */
        Out outBase = Out(1.0);
};

template <isQuantity In, isQuantity Out> class PID : public Controller<In, In, Out> {
        pidConfig<In, Out>& m_config;
        double m_integral = 0, m_prev_error = NAN;
        Out m_current;
    public:
        PID(pidConfig<In, Out>& config, const In initial_target)
            : m_config(config), Controller<In, In, Out>(initial_target), m_current(0) {}

        virtual Out update(const In& input) override { // todo: test this all
            double error = (this->m_target - input).convert(m_config.inBase);
            if (std::isnan(m_prev_error)) m_prev_error = error;
            double bbThreshold = m_config.bbangThreshold.convert(m_config.inBase);
            double max = m_config.outMax.convert(m_config.outBase);
            if ((!std::isnan(bbThreshold)) && (!std::isnan(m_config.bbangValue.internal())) && (error > bbThreshold)) {
                m_current = units::copysign(m_config.bbangValue, Number(error)); // bang bang
            } else {
                if ((error < 0) != (m_prev_error < 0))
                    m_integral *= m_config.integralCut; // cut down I if the sign of the error changes
                else if ((m_config.integralMax != In(0.0)) &&
                         (m_integral >= m_config.integralMax.convert(m_config.inBase)))
                    m_integral = std::copysign(m_config.integralMax.convert(m_config.inBase),
                                               m_integral); // cap I
                else m_integral += error;
                double output =
                    (error * m_config.kP) + (m_integral * m_config.kI) + ((m_prev_error - error) * m_config.kD);
                m_prev_error = error;
                if (!std::isnan(max) && fabs(output) > max) output = std::copysign(max, error); // cap output
                m_current = output * m_config.outBase;
            }
            return m_current;
        }

        virtual void reset() override {
            m_integral = 0;
            m_prev_error = NAN;
            m_current = Out(0);
        }
};
} // namespace controllers