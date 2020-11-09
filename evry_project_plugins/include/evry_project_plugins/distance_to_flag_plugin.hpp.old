#ifndef DISTANCE_TO_FLAG_PLUGIN_HPP
#define DISTANCE_TO_FLAG_PLUGIN_HPP

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Plugin.hh"
#include <ignition/math.hh>


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <evry_project_plugins/DistanceToFlag.h>

#include <random>


namespace gazebo{
  class DistanceToFlagPlugin: public ModelPlugin{
    public:
      DistanceToFlagPlugin();
      virtual ~DistanceToFlagPlugin();

    protected:
      virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

    private:
      physics::LinkPtr _link;
      ros::NodeHandle* _nh;
      ros::ServiceServer _dtf_service;

      std::default_random_engine _generator;
      std::normal_distribution<double> _distribution;

      bool distanceToFlag(evry_project_plugins::DistanceToFlag::Request& req, evry_project_plugins::DistanceToFlag::Response& res);
  };
}

#endif
