#include <dwa_local_planner/goal_align_cost_function.h>

using base_local_planner::Trajectory;

namespace dwa_local_planner {

bool GoalAlignCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  map_.resetPathDist();
  map_.setLocalGoal(*costmap_, target_poses_);
  global_pose_ = global_pose;
  scale_offset_ = -1.0;
  return true;
}


void GoalAlignCostFunction::setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y){

    double px = global_pose_.getOrigin().getX(), py = global_pose_.getOrigin().getY(), pth = tf::getYaw(global_pose_.getRotation());

    double sq_dist = (px - goal_x) * (px - goal_x) + (py - goal_y) * (py - goal_y);

    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    target_poses_ = orig_global_plan;
    double angle_to_goal = atan2(goal_y - py, goal_x - px);

    target_poses_.back().pose.position.x += xshift_ * cos(angle_to_goal);
    target_poses_.back().pose.position.y += xshift_ * sin(angle_to_goal);
}

} /* namespace dwa_local_planner */

 
