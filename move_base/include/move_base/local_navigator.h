#ifndef LOCAL_NAVIGATOR_H_
#define LOCAL_NAVIGATOR_H_
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
namespace move_base {
    class LocalNavigator {
        public: 
            LocalNavigator(tf::TransformListener& tf);
            ~LocalNavigator();
            
            bool setGlobalPlan( std::vector<geometry_msgs::PoseStamped>& global_plan );
            void start(){ controller_costmap_ros_->start(); }
            void stop(){ controller_costmap_ros_->stop(); }
            costmap_2d::Costmap2DROS* getCostmap(){ return controller_costmap_ros_; }
            
          /**
           * @brief  Performs a control cycle
           * @param goal A reference to the goal to pursue
           * @param global_plan A reference to the global plan being used
           * @return True if processing of the goal is done, false otherwise
           */
          bool executeCycle();
            /**
             * @brief  Publishes a velocity command of zero to the base
             */
            void publishZeroVelocity();
            
        private:
            void controlThread();
      double controller_frequency_, controller_patience_;
            ros::Publisher vel_pub_;
            tf::TransformListener& tf_;
            boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
            costmap_2d::Costmap2DROS* controller_costmap_ros_;
            pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;
      bool c_freq_change_;
    };

}; // namespace move_base

#endif