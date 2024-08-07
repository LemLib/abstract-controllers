#pragma once
#include "controller/controller.hpp"
#include "units/units.hpp"
#include <functional>
#include <type_traits>
#include <utility>

namespace controllers {

template <typename Target, typename In, typename Out> class AdditiveFusionController
    : public Controller<Target, In, Out> {
        Controller<Target, In, Out>&m_controller1, &m_controller2;
    public:
        AdditiveFusionController(Controller<Target, In, Out>& controller1, Controller<Target, In, Out>& controller2)
            : m_controller1(controller1), m_controller2(controller2),
              Controller<Target, In, Out>(controller1.getTarget()) {}

        virtual Out update(const In& input) override {
            return m_controller1.update(input) + m_controller2.update(input);
        }

        virtual void reset() override {
            m_controller1.reset();
            m_controller2.reset();
        }
};

template <typename Target, typename Target2, typename In, typename Mid, typename Out> class ChainedFusionController
    : public Controller<Target, In, Out> {
        Controller<Target, In, Mid>& m_controller1;
        Controller<Target2, Mid, Out>& m_controller2;
    public:
        ChainedFusionController(Controller<Target, In, Mid>& controller1, Controller<Target2, Mid, Out>& controller2)
            : m_controller1(controller1), m_controller2(controller2),
              Controller<Target, In, Out>(controller1.getTarget()) {}

        virtual Out update(const In& input) override { return m_controller2.update(m_controller1.update(input)); }

        virtual void reset() override {
            m_controller1.reset();
            m_controller2.reset();
        }
};

template <typename Target, typename In, typename Out, typename Target1, typename In1, typename Out1, typename Target2,
          typename In2, typename Out2>
class FunctionalChainedController : public Controller<Target, In, Out> {
        using Function = std::function<Out(In, Controller<Target1, In1, Out1>&, Controller<Target2, In2, Out2>&)>;
        Controller<Target1, In1, Out1>& m_controller1;
        Controller<Target2, In2, Out2>& m_controller2;
        Function m_func;
    public:
        FunctionalChainedController(Controller<Target1, In1, Out1>& controller1,
                                    Controller<Target2, In2, Out2>& controller2, Function func, Target initial_target)
            : m_controller1(controller1), m_controller2(controller2), m_func(func), Controller<Target, In, Out>(initial_target) {}

        virtual Out update(const In& input) override { return m_func(input, m_controller1, m_controller2); }

        virtual void reset() override {
            m_controller1.reset();
            m_controller2.reset();
        }
};
} // namespace controllers