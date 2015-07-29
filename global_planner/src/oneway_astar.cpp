/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: David V. Lu!!
 *********************************************************************/
#include<global_planner/oneway_astar.h>

namespace global_planner {

OneWayAStar::OneWayAStar(PotentialCalculator* p_calc, int xs, int ys) :
        AStarExpansion(p_calc, xs, ys), got_map_(false) {
    ros::NodeHandle g_nh;
    map_sub_ = g_nh.subscribe("/one_way_map", 1, &OneWayAStar::incomingMap, this);
    ros::Rate r(10);
    while (!got_map_ && g_nh.ok())
    {
        ROS_DEBUG_THROTTLE(2.0, "Waiting for oneway map...");
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("One way map recieved.");
}

void OneWayAStar::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
    grid_ = *new_map;
    got_map_ = true;
    int c[260];
    
    for(int i=0;i<260;i++){
        c[i] = 0;
    }
    
    for(int i=0;i<grid_.data.size();i++){
        unsigned char v = grid_.data[i];
        c[ v ] += 1;
    }
    
    for(int i=0;i<260;i++){
        if(c[i]>0){
            ROS_INFO("%03d %03x | %d", i, i, c[i]);
        }    
    }
}

bool OneWayAStar::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear();
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
            return true;
            
        unsigned char value = grid_.data[i];    

        if(value & 0x10)
            add(costs, potential, potential[i], i + 1, end_x, end_y);
        if(value & 0x20)    
            add(costs, potential, potential[i], i - 1, end_x, end_y);
        if(value & 0x40)    
            add(costs, potential, potential[i], i + nx_, end_x, end_y);
        if(value & 0x80)    
            add(costs, potential, potential[i], i - nx_, end_x, end_y);
    }

    return false;
}

} //end namespace global_planner
