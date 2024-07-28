#pragma once
#include "controller/controller.hpp"
#include "units/units.hpp"
#include <functional>
#include <type_traits>
#include <utility>

namespace controllers {

template <typename Target, typename In, typename Out,
          typename = std::enable_if<Isomorphic<typeof(std::declval<Out> + std::declval<Out>),
                                               Out>>> // todo is this really necessary? Probably
class AdditiveFusionController : public Controller<Target, In, Out> {
        static_assert(std::declval<Out>() + std::declval<Out>());
        Controller<Target, In, Out>&controller1, &controller2;
        Out current;
    public:
        AdditiveFusionController(Controller<Target, In, Out>& controller1, Controller<Target, In, Out>& controller2)
            : controller1(controller1), controller2(controller2) {}

        virtual Out getOut() const override { return current; }

        virtual Out step(In setting) override {
            current = controller1.step(setting) + controller2.step(setting);
            return current;
        }
};

template <typename Target, typename In, typename Mid, typename Out> class ChainedFusionController
    : public Controller<Target, In, Out> {
        Controller<Target, In, Mid>& controller1;
        Controller<Target, Mid, Out>& controller2;
    public:
        ChainedFusionController(Controller<Target, In, Mid>& controller1, Controller<Target, Mid, Out>& controller2)
            : controller1(controller1), controller2(controller2) {}

        virtual Out update(In input) override { return controller2.step(controller1.step(input)); }
};

template <typename Target, typename In, typename Out, typename Target1, typename In1, typename Out1, typename Target2,
          typename In2, typename Out2>
class CustomChainedController : public Controller<Target, In, Out> {
        using Function = std::function<Out(In, Controller<Target1, In1, Out1>&, Controller<Target2, In2, Out2>&)>;
        Controller<Target1, In1, Out1>& controller1;
        Controller<Target2, In2, Out2>& controller2;
        Function func;
    public:
        CustomChainedController(Controller<Target1, In1, Out1>& controller1,
                                Controller<Target2, In2, Out2>& controller2, Function func)
            : controller1(controller1), controller2(controller2), func(func) {}

        virtual Out update(In input) override { return func(input, controller1, controller2); }
};
} // namespace controllers