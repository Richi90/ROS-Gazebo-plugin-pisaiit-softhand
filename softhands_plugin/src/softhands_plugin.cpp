#include <softhands_plugin/softhands_plugin.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <string>
#include <ros/ros.h>

using namespace gazebo;
using namespace std;

/*


*/

void softhandsPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  int argc = 0;
  char **argv;
  ros::init(argc, argv, "softhands_Plugin");

  this->model = _parent;

  // Make sure the ROS node for Gazebo has already been initialized                                                                                    
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Retrieve joint identifier and control mode from urdf tags
  this->joint_name =_sdf->GetElement("joint")->Get<string>();
  this->ns_name =_sdf->GetElement("namespace")->Get<string>();
  this->K =_sdf->GetElement("stiffness")->Get<double>();

  // Everything-is-fine message
  std::string ok_msg = "Softhands plugin on " + joint_name + " started!";
  ROS_WARN_STREAM(ok_msg);

  // Retrieve joint
  this->joint = this->model->GetJoint(joint_name);

  // Compose the name of the publisher, removing the namespace in order to be coherent with the synergy plugin
  int len_ns = ns_name.length();
  std::string joint_name_wo_ns = joint_name.erase(0, len_ns+1);
  cmd_sub_name = ns_name + "/" + joint_name_wo_ns + "/command";
  //cmd_pub_name = ns_name + "/" + joint_name + "/leggimi";

  // Subscribers and Publishers for the joint states and command
  sub = n.subscribe(cmd_sub_name, 10, &softhandsPlugin::getRef_callback, this);
  //pub = n.advertise<std_msgs::Float64>(cmd_pub_name, 500);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&softhandsPlugin::OnUpdate, this, _1));  

    // DEBUG
/*  std::cout << cmd_ref1_name << std::endl;
  std::cout << cmd_ref2_name << std::endl;*/
}

// Subscriber callbacks references
void softhandsPlugin::getRef_callback(const std_msgs::Float64& val)
{
    this->joint_ref = val;
}

// Main update function
void softhandsPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  // Retrieve joint actual position
  this->q = this->joint->GetAngle(0).Radian();
  this->dq = this->joint->GetVelocity(0);

  // Compute the elastic torque
  this->joint_tau.data = this->K*(this->joint_ref.data - this->q) - this->Damp*this->dq;

  // Set to the joint elastic torque
  this->joint->SetForce(0, this->joint_tau.data);

  //pub.publish(this->joint_tau);
}