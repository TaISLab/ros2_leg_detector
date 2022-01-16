/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

// ROS related Headers
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Custom Messages related Headers
#include "leg_detector_msgs/msg/leg.hpp"
#include "leg_detector_msgs/msg/step.hpp"
#include "leg_detector_msgs/msg/step_array.hpp"

#include <fstream>

class PlotSteps : public rclcpp::Node{

public:
    PlotSteps() : Node("steps_plotter"){
        id_counter = 0;
        // declare parameters
        this->declare_parameter("steps_topic_name");
        this->declare_parameter("marker_display_lifetime");
        this->declare_parameter("speed_dead_zone");

        // default values
        this->get_parameter_or("steps_topic_name", steps_topic_name_, std::string("/detected_steps"));
        this->get_parameter_or("marker_display_lifetime", marker_display_lifetime_, 0.01);
        this->get_parameter_or("speed_dead_zone", speed_dead_zone_, 0.05);
        
        
        //Print  parameters
        RCLCPP_INFO(this->get_logger(), "Node parameters: ");
        RCLCPP_INFO(this->get_logger(), "steps_topic_name: %s", steps_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "marker_display_lifetime: %.2f", marker_display_lifetime_);
        RCLCPP_INFO(this->get_logger(), "speed_dead_zone: %.2f", speed_dead_zone_);

        // pub/sub
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        this->markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 20);

        this->steps_topic_sub_ = this->create_subscription<leg_detector_msgs::msg::StepArray>(steps_topic_name_, default_qos, std::bind(&PlotSteps::stepsCallback, this, std::placeholders::_1));

    }

private:
    int id_counter;
    std::string steps_topic_name_;
    double marker_display_lifetime_;
    double speed_dead_zone_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markers_pub_;
    rclcpp::Subscription<leg_detector_msgs::msg::StepArray>::SharedPtr steps_topic_sub_;
    visualization_msgs::msg::Marker leg_l_marker, leg_r_marker;

    void stepsCallback(const leg_detector_msgs::msg::StepArray::SharedPtr stepsArr){


        if (stepsArr->steps[0].leg.confidence>0){
            
            leg_l_marker = fill_marker(stepsArr->steps[0], stepsArr->header, 0, true);            
            markers_pub_->publish(leg_l_marker);
        }
        
        if (stepsArr->steps[0].leg.confidence>0){
            leg_r_marker = fill_marker(stepsArr->steps[1], stepsArr->header, 1, false);
            markers_pub_->publish(leg_r_marker);
        }

    }

    visualization_msgs::msg::Marker fill_marker(leg_detector_msgs::msg::Step step, std_msgs::msg::Header header, int id, bool left) {
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.ns = "LEGS";
        m.id = id;
        if (left)
            m.type = m.SPHERE;
        else
            m.type = m.CUBE;
        m.pose.position.x = step.leg.position.x;
        m.pose.position.y = step.leg.position.y;
        m.pose.position.z = 0.2;
        m.scale.x = 0.13;
        m.scale.y = 0.13;
        m.scale.z = 0.13;
        m.color.a = step.leg.confidence;
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 0;            
        if (step.speed>speed_dead_zone_){
            m.color.b = 1;
        } else if (step.speed<(-1.0*speed_dead_zone_)){
            m.color.r = 1;
        }

        m.lifetime = rclcpp::Duration(marker_display_lifetime_);
        return m;
    }




};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlotSteps>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
