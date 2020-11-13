#include <evry_project_plugins/distance_to_flag_plugin.hpp>

namespace gazebo{
  DistanceToFlagPlugin::DistanceToFlagPlugin(): _distribution(0, 0.05){
  }

  DistanceToFlagPlugin::~DistanceToFlagPlugin(){
    _nh->shutdown();
    delete _nh;
  }

  void DistanceToFlagPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf){
    int nbFlags = sdf->GetElement("nbFlags")->Get<int>();

    //get the flags poses and store it in the list _flags_poses
    for(int i = 1; i < nbFlags+1; i++){
      ignition::math::Pose3d model_pose = world->ModelByName("flag_" + std::to_string(i))->WorldPose();
      geometry_msgs::Pose2D flag_pose;
      flag_pose.x = model_pose.Pos().X();
      flag_pose.y = model_pose.Pos().Y();
      _flags_poses.push_back(flag_pose);
    }

    _nh = new ros::NodeHandle();
    _dtf_service = _nh->advertiseService("distanceToFlag", &DistanceToFlagPlugin::distanceToFlag, this);
  }

  float DistanceToFlagPlugin::getMinDist(const geometry_msgs::Pose2D& msg, int* imin){
    float min_dist = FLT_MAX;
    float dx, dy, dist;
    for(int i = 0; i < _flags_poses.size(); i++){
      dx = _flags_poses[i].x - msg.x;
      dy = _flags_poses[i].y - msg.y;
      dist = std::sqrt(dx*dx + dy*dy);
      if(dist < min_dist){
        min_dist = dist;
        *imin = i+1; //the flag id starts at 1 not 0 (their gazebo model name)
      }
    }
    return min_dist;
  }

  bool DistanceToFlagPlugin::distanceToFlag(evry_project_plugins::DistanceToFlag::Request& req, evry_project_plugins::DistanceToFlag::Response& res){
    int imin = 1;
    res.distance = getMinDist(req.agent_pose, &imin) + _distribution(_generator);
    res.id_flag = imin;
    return true;
  }

  GZ_REGISTER_WORLD_PLUGIN(DistanceToFlagPlugin)
}
