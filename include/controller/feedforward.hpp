#pragma once
#include "controller/controller.hpp"
#include "units/units.hpp"
#include <functional>

namespace controllers {
template <isQuantity In, isQuantity Out> class LinearFeedForward : public Controller<In, In, Out> {
        using Div = Divided<In, Out>::Self;
        Div m_kF;
        Out m_b;
    public:
        LinearFeedForward(const In initial_target, const Div kF, const Out b = 0)
            : m_kF(kF), m_b(b), Controller<In, In, Out>(initial_target) {}

        virtual Out update(In setting) override { return m_kF * setting + m_b; }

        virtual void reset() override {
            // do nothing lmao
        }
};

template <isQuantity In, isQuantity Out> class FunctionalFeedForward : public Controller<In, In, Out> {
        std::function<Out(In)> m_func;
    public:
        FunctionalFeedForward(const In initial_target, std::function<Out(In)> func)
            : m_func(func), Controller<In, In, Out>(initial_target) {}

        virtual Out update(In setting) override { return m_func(setting); }

        virtual void reset() override {
            // do nothing lmao
        }
};
} // namespace controllers