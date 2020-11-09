#include <evry_project_plugins/distance_to_flag_plugin.hpp>

namespace gazebo{
  DistanceToFlagPlugin::DistanceToFlagPlugin(): _distribution(0, 0.1){
  }

  DistanceToFlagPlugin::~DistanceToFlagPlugin(){
    _nh->shutdown();
    delete _nh;
  }

  void DistanceToFlagPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf){
    int id = sdf->GetElement("id")->Get<int>();

    _link = model->GetLink();

    _nh = new ros::NodeHandle();
    _dtf_service = _nh->advertiseService("dist_to_flag_number_" + std::to_string(id), &DistanceToFlagPlugin::distanceToFlag, this);
  }

  bool DistanceToFlagPlugin::distanceToFlag(evry_project_plugins::DistanceToFlag::Request& req, evry_project_plugins::DistanceToFlag::Response& res){
    ignition::math::Pose3d flag_pose = _link->WorldPose();
    float dx = flag_pose.Pos().X() - req.agent_pose.x;
    float dy = flag_pose.Pos().Y() - req.agent_pose.y;
    res.distance = std::sqrt(dx*dx + dy*dy) + _distribution(_generator);
    return true;
  }

  GZ_REGISTER_MODEL_PLUGIN(DistanceToFlagPlugin)
}
