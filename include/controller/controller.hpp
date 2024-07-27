#pragma once

namespace controllers {
/**
 * @brief Abstract controller class
 *
 * This is used to allow the user to define their own controllers to be used
 * with LemLib. It's templated to allow for any input/output type.
 *
 * @tparam Target the type of the target value
 * @tparam In the type of the input
 * @tparam Out the type of the output
 */
template <typename Target, typename In, typename Out> class Controller {
public:
  /**
   * @brief set the target value of the controller
   *
   * @param target the target value
   */
  virtual void setTarget(const Target &target) { this->m_target = target; }

  /**
   * @brief Get the target value of the controller
   *
   * @param target
   */
  virtual Target getTarget() const { return m_target; }

  /**
   * @brief Update the controller
   *
   * @param in the input to the controller
   *
   * @return out the output of the controller
   */
  virtual Out update(const In &input) = 0;
  /**
   * @brief reset the controller
   *
   */
  virtual void reset() = 0;
  /**
   * @brief Destroy the Controller object
   *
   */
  virtual ~Controller(){};

protected:
  Target m_target; /** the target value of the controller */
};
} // namespace controllers