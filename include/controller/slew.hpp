#pragma once
#include "controller/controller.hpp"
#include "units/units.hpp"

namespace controllers {
template <isQuantity In, isQuantity Target, isQuantity Out> class Slewed : public Controller<In, Target, Out> {
    private:
        Out m_max, m_prev;
        Controller<In, Target, Out>* m_internal;
    public:
        Slewed(Controller<In, Target, Out>& internal, Out max)
            : m_internal(&internal), m_max(max), m_prev(0), Controller<In, Target, Out>(In(0)) {}

        virtual Out update(const In& input) override {
            Out current = m_internal->update(input);
            if (units::abs(current - m_prev) > m_max) current = m_prev + units::copysign(m_max, current);
            m_prev = current;
            return current;
        }

        virtual void reset() override { m_internal->reset(); }

        virtual void setTarget(const Target& target) override { m_internal->setTarget(target); }

        virtual Target getTarget() const override { return m_internal->getTarget(); }
};
} // namespace controllers