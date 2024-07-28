#pragma once
#include "controller/controller.hpp"
#include "units/units.hpp"

namespace controllers {
template <isQuantity In, isQuantity Out> class Slewed : public Controller<In, In, Out> {
    private:
        Out m_max, m_prev;
        Controller<In, In, Out>* m_internal;
    public:
        Slewed(Controller<In, In, Out>& internal, Out max)
            : m_internal(&internal), m_max(max), m_prev(0), Controller<In, In, Out>(In(0)) {}

        virtual Out update(const In& input) override {
            Out current = m_internal->update(input);
            if (units::abs(current - m_prev) > m_max) current = m_prev + units::copysign(m_max, current);
            current = m_prev;
        }

        virtual void reset() override { m_internal->reset(); }

        virtual void setTarget(const In& target) override { m_internal->setTarget(target); }

        virtual In getTarget() const override { return m_internal->getTarget(); }
};
} // namespace controllers