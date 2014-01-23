#ifndef MOVE_BASE_STATE_MACHINE_
#define MOVE_BASE_STATE_MACHINE_

#include<move_base/global_navigator.h>
#include<move_base/local_navigator.h>

namespace move_base {

  class StateMachine{
    public:
      virtual void initialize(tf::TransformListener* tf, GlobalNavigator* planner, LocalNavigator* controller)
      {
         tf_ = tf;
         planner_ = planner;
         controller_ = controller;
      }

      virtual void reset() = 0;
      virtual std::string executeCycle() = 0;

    protected:
      StateMachine(){}
      GlobalNavigator* planner_;
      LocalNavigator* controller_;
      tf::TransformListener* tf_;
  };
};

#endif