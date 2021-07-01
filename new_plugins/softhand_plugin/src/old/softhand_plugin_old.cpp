#include <softhand_plugin/softhand_plugin.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> 
#include <sensor_msgs/JointState.h>

using namespace gazebo;
using namespace std;

/*
  TO ADD DESCRIPTION
*/

// Saturation for input torque
double SoftHandPlugin::saturate(double val, double max_val)
{
    if (val > max_val) {
        val = max_val;
    }
    if (val < -max_val) {
        val = -max_val;
    }
    return val;
}

// Update function of the motor class
void SoftHandPlugin::DCMotor::OnUpdate(const double & dT, double & tauMot, const double & tauLoad)
{
    tauMot = saturate(tauMot, tauMax);

    const double K = 1e3;

    if  (pos - w < -tauFric/K){
        w = pos + tauFric/K;
    }
    else if (pos - w > tauFric/K){
        w = pos - tauFric/K;
    }
    // else w = w;

    double tauFricNow = K *(pos - w);

    effort = - D*vel + tauMot + tauLoad + tauFricNow;
    double acc = effort / J;

    vel = vel + acc * dT;

    pos = pos + vel * dT;

    if(pos < minPos){
        pos = minPos;
        vel = 0;
        effort = std::numeric_limits<double>::quiet_NaN();
    }
    else if(pos > maxPos){
        pos = maxPos;
        vel = 0;
        effort = std::numeric_limits<double>::quiet_NaN();
    }
}

// Update function of the PID controller class
void SoftHandPlugin::PIDController::OnUpdate(const double & dT, const double & pos, const double & vel)
{
    double e = posRef - pos;
    double de = velRef - vel;
    errInt += I * e * dT;

    effort = errInt + P * e + D * de;
}

// Subscriber callbacks references
void SoftHandPlugin::getRef_callback(const std_msgs::Float64& val)
{

    ref = val.data;
}

// Subscribers fingers commands (in case of fully actuation modes)
void SoftHandPlugin::getThumbCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_thumb = val;
}
void SoftHandPlugin::getIndexCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_index = val;
}
void SoftHandPlugin::getMiddleCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_middle = val;
}
void SoftHandPlugin::getRingCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_ring = val;
}
void SoftHandPlugin::getLittleCmd_callback(const std_msgs::Float64MultiArray& val){
  
    cmd_little = val;
}

// Subscriber callback for external torque
void SoftHandPlugin::getExtTau_callback(const std_msgs::Float64& e_tau){
    
    ext_tau = e_tau.data;
}

// Function to  retrieve the joint finger position and velocity
void SoftHandPlugin::GetFingersState(){
    for (int i = 0; i < 4; ++i)
    {
      if (i==3)
      {
        q_fingers[0][i] = 0;    // thumb finger has one less joint
        dq_fingers[0][i] = 0;   // .... as above!
      }
      else{
        q_fingers[0][i] = thumb_joint[i]->Position(0);
        dq_fingers[0][i] = thumb_joint[i]->GetVelocity(0);
      }
      q_fingers[1][i] = index_joint[i]->Position(0);
      dq_fingers[1][i] = index_joint[i]->GetVelocity(0);
      
      q_fingers[2][i] = middle_joint[i]->Position(0);
      dq_fingers[2][i] = middle_joint[i]->GetVelocity(0);

      q_fingers[3][i] = ring_joint[i]->Position(0);
      dq_fingers[3][i] = ring_joint[i]->GetVelocity(0);

      q_fingers[4][i] = little_joint[i]->Position(0);
      dq_fingers[4][i] = little_joint[i]->GetVelocity(0);
    }
}

// Function to set the fingers torque in Gazebo
void SoftHandPlugin::SetFingersTorque(double tauFingers[][4]){
    for (int i = 0; i < 4; ++i)
    { 
      // thumb
      if (i==3)
      {
        // do nothing!
      }else{
        thumb_joint[i]-> SetForce(0, tauFingers[0][i]);
      }
      // index
      index_joint[i]-> SetForce(0, tauFingers[1][i]);
      // middle
      middle_joint[i]-> SetForce(0, tauFingers[2][i]);
      // ring
      ring_joint[i]-> SetForce(0, tauFingers[3][i]);      
      // little
      little_joint[i]-> SetForce(0, tauFingers[4][i]);
    }
}

// Soft Synergies control update function
void SoftHandPlugin::OnUpdateSoftSyn(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // compute old elastic torque & update joint effort
    double tauEl_old[5][4];
    for (int i = 0; i < 4; ++i)
    { 
      // thumb
      if (i==3)
      {
        tauEl_old[0][i] = 0; // thumb finger has one less joint
      }else{
        tauEl_old[0][i] = spring_k*(thumb_syn[i]*ref - q_fingers[0][i]);
      }
      // index
      tauEl_old[1][i] = spring_k*(index_syn[i]*ref - q_fingers[1][i]);
      // middle
      tauEl_old[2][i] = spring_k*(middle_syn[i]*ref - q_fingers[2][i]);
      // ring
      tauEl_old[3][i] = spring_k*(ring_syn[i]*ref - q_fingers[3][i]);
      // little
      tauEl_old[4][i] = spring_k*(little_syn[i]*ref - q_fingers[4][i]);
    }
    SetFingersTorque(tauEl_old);

    // compute new elastic torque & update joint effort
    double tauEl[5][4];
    for (int i = 0; i < 4; ++i)
    { 
      // thumb
      if (i==3)
      {
        tauEl[0][i] = 0; // thumb finger has one less joint
      }else{
        tauEl[0][i] = spring_k*(thumb_syn[i]*ref - q_fingers[0][i]);
      }
      // index
      tauEl[1][i] = spring_k*(index_syn[i]*ref - q_fingers[1][i]);
      // middle
      tauEl[2][i] = spring_k*(middle_syn[i]*ref - q_fingers[2][i]);
      // ring
      tauEl[3][i] = spring_k*(ring_syn[i]*ref - q_fingers[3][i]);
      // little
      tauEl[4][i] = spring_k*(little_syn[i]*ref - q_fingers[4][i]);
    }
    SetFingersTorque(tauEl);

    // publish relevant topics 
    SoftHandPlugin::Publish(tauEl);

    // update timer
    oldTime = t;
}

// Compute current synergies value from the finger positions
double SoftHandPlugin::ComputeActualSyn(){
    double synActualPos = 0;
    for (int i = 0; i < 4; ++i)
    { 
      if (i==3){        // thumb finger has one less joint
        synActualPos += index_syn[i]*q_fingers[1][i] + middle_syn[i]*q_fingers[2][i] + 
                        ring_syn[i]*q_fingers[3][i] + little_syn[i]*q_fingers[4][i]; 
      }else{
        synActualPos += thumb_syn[i]*q_fingers[0][i] + index_syn[i]*q_fingers[1][i] + middle_syn[i]*q_fingers[2][i] +
                        ring_syn[i]*q_fingers[3][i] + little_syn[i]*q_fingers[4][i];
      }

    }
    return synActualPos;
}

// Adaptive Synergies control update function,  with torque reference
void SoftHandPlugin::OnUpdateAdaptSynTau(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // assign reference to synergy motor torque
    double tauM = ref;

    // compute old elastic torque & update joint effort
    double tauFing[5][4];
    for (int i = 0; i < 4; ++i)
    { 
      // thumb
      if (i==3)
      {
        tauFing[0][i] = 0; // thumb finger has one less joint
      }else{
        tauFing[0][i] = n_syn[i]*tauM - spring_k*q_fingers[0][i];
      }
      // index
      tauFing[1][i] = n_syn[i]*tauM - spring_k*q_fingers[1][i];
      // middle
      tauFing[2][i] = n_syn[i]*tauM - spring_k*q_fingers[2][i];
      // ring
      tauFing[3][i] = n_syn[i]*tauM - spring_k*q_fingers[3][i];
      // little
      tauFing[4][i] = n_syn[i]*tauM - spring_k*q_fingers[4][i];
    }
    SetFingersTorque(tauFing);

    // publish relevant topics
    mot.pos = ComputeActualSyn();
    mot.effort = tauM;
    SoftHandPlugin::Publish(tauFing);
    SoftHandPlugin::PublishMotor();

    // update timer
    oldTime = t;
}

// // Adaptive Synergies control update function, with position reference 
// void SoftHandPlugin::OnUpdateAdaptSynPos(const common::UpdateInfo & info){

//     // retrieve simulation time
//     common::Time t = info.simTime;
//     double dT = (t - oldTime).Double();

//     // populate fingers state
//     GetFingersState();

//     // update controllers
//     ctrl.posRef = ref;
//     ctrl.OnUpdate(dT, mot.pos, mot.vel);

//     // compute singery value from finger position
//     double synActualPos_old = 0;
//     for (int i = 0; i < 4; ++i)
//     { 
//       // thumb
//       if (i==3)
//       {
//         synActualPos_old += index_syn[i]*q_fingers[1][i] + middle_syn[i]*q_fingers[2][i] +
//                         ring_syn[i]*q_fingers[3][i] + little_syn[i]*q_fingers[4][i]; // thumb finger has one less joint
//       }else{
//         synActualPos_old += thumb_syn[i]*q_fingers[0][i] + index_syn[i]*q_fingers[1][i] + middle_syn[i]*q_fingers[2][i] +
//                         ring_syn[i]*q_fingers[3][i] + little_syn[i]*q_fingers[4][i];
//       }

//     }

//     // compute synergy motor torque
//     double tauM_old = spring_k_tendon*(mot.pos - synActualPos_old);
//     tauM_old = saturate(tauM_old, mot.tauMax);

//     // compute old elastic torque & update joint effort
//     double tauFing_old[5][4];
//     for (int i = 0; i < 4; ++i)
//     { 
//       // thumb
//       if (i==3)
//       {
//         tauFing_old[0][i] = 0; // thumb finger has one less joint
//       }else{
//         tauFing_old[0][i] = n_syn[i]*tauM_old - spring_k*q_fingers[0][i];
//       }
//       // index
//       tauFing_old[1][i] = n_syn[i]*tauM_old - spring_k*q_fingers[1][i];
//       // middle
//       tauFing_old[2][i] = n_syn[i]*tauM_old - spring_k*q_fingers[2][i];
//       // ring
//       tauFing_old[3][i] = n_syn[i]*tauM_old - spring_k*q_fingers[3][i];
//       // little
//       tauFing_old[4][i] = n_syn[i]*tauM_old - spring_k*q_fingers[4][i];
//     }
//     SetFingersTorque(tauFing_old);

//     //update motor
//     mot.OnUpdate(dT, ctrl.effort, tauM_old);

//     // populate fingers state
//     GetFingersState();

//     // compute singery value from finger position
//     double synActualPos = 0;
//     for (int i = 0; i < 4; ++i)
//     { 
//       // thumb
//       if (i==3)
//       {
//         synActualPos += index_syn[i]*q_fingers[1][i] + middle_syn[i]*q_fingers[2][i] +
//                         ring_syn[i]*q_fingers[3][i] + little_syn[i]*q_fingers[4][i]; // thumb finger has one less joint
//       }else{
//         synActualPos += thumb_syn[i]*q_fingers[0][i] + index_syn[i]*q_fingers[1][i] + middle_syn[i]*q_fingers[2][i] +
//                         ring_syn[i]*q_fingers[3][i] + little_syn[i]*q_fingers[4][i];
//       }

//     }
//     std::cout << "\t Meas synergy value = " << synActualPos << std::flush;

//     // compute synergy motor torque
//     double tauM = spring_k_tendon*(mot.pos - synActualPos);
//     tauM = saturate(tauM, mot.tauMax);
//     std::cout << "\t Meas synergy torque = " << tauM << std::flush;

//     // compute old elastic torque & update joint effort
//     double tauFing[5][4];
//     for (int i = 0; i < 4; ++i)
//     { 
//       // thumb
//       if (i==3)
//       {
//         tauFing[0][i] = 0; // thumb finger has one less joint
//       }else{
//         tauFing[0][i] = n_syn[i]*tauM - spring_k*q_fingers[0][i];
//       }
//       // index
//       tauFing[1][i] = n_syn[i]*tauM - spring_k*q_fingers[1][i];
//       // middle
//       tauFing[2][i] = n_syn[i]*tauM - spring_k*q_fingers[2][i];
//       // ring
//       tauFing[3][i] = n_syn[i]*tauM - spring_k*q_fingers[3][i];
//       // little
//       tauFing[4][i] = n_syn[i]*tauM - spring_k*q_fingers[4][i];
//     }
//     SetFingersTorque(tauFing);

//     // publish relevant topics 
//     SoftHandPlugin::Publish(tauFing);
//     SoftHandPlugin::PublishMotor();

//     // update timer
//     oldTime = t;
// }

// Adaptive Synergies control update function, with position reference 
void SoftHandPlugin::OnUpdateAdaptSynPos(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // update controllers
    ctrl.posRef = ref;
    ctrl.OnUpdate(dT, mot.pos, mot.vel);

    // compute singery value from finger position
    double synActualPos = ComputeActualSyn();
    // for (int i = 0; i < 4; ++i)
    // { 
    //   // thumb
    //   if (i==3)
    //   {
    //     synActualPos += index_syn[i]*q_fingers[1][i] + middle_syn[i]*q_fingers[2][i] +
    //                     ring_syn[i]*q_fingers[3][i] + little_syn[i]*q_fingers[4][i]; // thumb finger has one less joint
    //   }else{
    //     synActualPos += thumb_syn[i]*q_fingers[0][i] + index_syn[i]*q_fingers[1][i] + middle_syn[i]*q_fingers[2][i] +
    //                     ring_syn[i]*q_fingers[3][i] + little_syn[i]*q_fingers[4][i];
    //   }

    // }
    std::cout << "\t Meas synergy value = " << synActualPos << std::flush;

    // compute synergy motor torque
    double tauM = spring_k_tendon*(mot.pos - synActualPos);
    // tauM = saturate(tauM, mot.tauMax);
    std::cout << "\t Meas synergy torque = " << tauM << std::flush;

    // compute old elastic torque & update joint effort
    double tauFing[5][4];
    for (int i = 0; i < 4; ++i)
    { 
      // thumb
      if (i==3)
      {
        tauFing[0][i] = 0; // thumb finger has one less joint
      }else{
        tauFing[0][i] = n_syn[i]*tauM - spring_k*q_fingers[0][i];
      }
      // index
      tauFing[1][i] = n_syn[i]*tauM - spring_k*q_fingers[1][i];
      // middle
      tauFing[2][i] = n_syn[i]*tauM - spring_k*q_fingers[2][i];
      // ring
      tauFing[3][i] = n_syn[i]*tauM - spring_k*q_fingers[3][i];
      // little
      tauFing[4][i] = n_syn[i]*tauM - spring_k*q_fingers[4][i];
    }
    SetFingersTorque(tauFing);

    //update motor
    mot.OnUpdate(dT, ctrl.effort, tauM);

    // publish relevant topics 
    SoftHandPlugin::Publish(tauFing);
    SoftHandPlugin::PublishMotor();

    // update timer
    oldTime = t;
}

// Direct Torque control update function
void SoftHandPlugin::OnUpdateFingTau(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // compute old elastic torque & update joint effort
    double tauFingers[5][4];
    for (int i = 0; i < 4; ++i)
    { 
      // thumb
      if (i==3)
      {
        tauFingers[0][i] = 0; // thumb finger has one less joint
      }else{
        tauFingers[0][i] = cmd_thumb.data[i];
      }
      // index
      tauFingers[1][i] = cmd_index.data[i];
      // middle
      tauFingers[2][i] = cmd_middle.data[i];
      // ring
      tauFingers[3][i] = cmd_ring.data[i];
      // little
      tauFingers[4][i] = cmd_little.data[i];
    }
    SetFingersTorque(tauFingers);

    // publish relevant topics 
    SoftHandPlugin::Publish(tauFingers);

    // update timer
    oldTime = t;  
}

// Position control update function
void SoftHandPlugin::OnUpdateFingPos(const common::UpdateInfo & info){

    // retrieve simulation time
    common::Time t = info.simTime;
    double dT = (t - oldTime).Double();

    // populate fingers state
    GetFingersState();

    // compute fingers torque using the PID controllers
    double tauFingers[5][4]; 
    for (int i = 0; i < 4; ++i)
    { 
      // thumb
      if (i==3)
      {
        // do nothing!
      }else{
        pid_thumb[i].posRef = cmd_thumb.data[i];
        pid_thumb[i].OnUpdate(dT, q_fingers[0][i], dq_fingers[0][i]);
        tauFingers[0][i] = pid_thumb[i].effort;
      }
      // index
      pid_index[i].posRef = cmd_index.data[i];
      pid_index[i].OnUpdate(dT, q_fingers[1][i], dq_fingers[1][i]);
      tauFingers[1][i] = pid_index[i].effort;
      // middle
      pid_middle[i].posRef = cmd_middle.data[i];
      pid_middle[i].OnUpdate(dT, q_fingers[2][i], dq_fingers[2][i]);
      tauFingers[2][i] = pid_middle[i].effort;
      // ring
      pid_ring[i].posRef = cmd_ring.data[i];
      pid_ring[i].OnUpdate(dT, q_fingers[3][i], dq_fingers[3][i]);
      tauFingers[3][i] = pid_ring[i].effort;
      // little
      pid_little[i].posRef = cmd_little.data[i];
      pid_little[i].OnUpdate(dT, q_fingers[4][i], dq_fingers[4][i]);
      tauFingers[4][i] = pid_little[i].effort;
    }
    SetFingersTorque(tauFingers);

    // publish relevant topics 
    SoftHandPlugin::Publish(tauFingers);

    // update timer
    oldTime = t;  
}

// Main update function
void SoftHandPlugin::OnUpdate(const common::UpdateInfo & info){
    switch(operationMode){
    case(SoftSynergies):            // operationMode = 0
        OnUpdateSoftSyn(info);
        std::cout << "\rI'm running SOFT SYNERGIES " << std::flush;
        break;
    case(AdaptiveSynergiesTorque):        // operationMode = 1
        OnUpdateAdaptSynTau(info);
        std::cout << "\rI'm running ADAPTIVE (tau) SYNERGIES " << std::flush;
        break;
    case(AdaptiveSynergiesPosition):        // operationMode = 1
        OnUpdateAdaptSynPos(info);
        std::cout << "\rI'm running ADAPTIVE (pos) SYNERGIES " << std::flush;
        break;
    case(FingerTorques):            // operationMode = 2 
        OnUpdateFingTau(info);
        std::cout << "\rI'm running TORQUE CONTROL (of the fingers) " << std::flush;
        break;
    case(FingerPositions):          // operationMode = 3 
        OnUpdateFingPos(info);
        std::cout << "\rI'm running POSITION CONTROL (of the fingers) " << std::flush;
        break;
    }
}

// Publish information relative to the fingers
void SoftHandPlugin::Publish(double tauFing[][4])
{
  // Publish whole joint state
  if (flag_pub_state)
  {
    // thumb
    int thumb_idx[3] = {0, 1, 3};
    for (int i = 0; i < 3; ++i)
    {
      thumb_state.name[i] = "Thumb_" + finger_part_names[thumb_idx[i]];
      thumb_state.position[i] = q_fingers[0][i];
      thumb_state.velocity[i] = dq_fingers[0][i];
      thumb_state.effort[i] = tauFing[0][i];
    }
    pub_thumb_state.publish(thumb_state);

    // other fingers
    for (int i = 0; i < 4; ++i)
    {
      index_state.name[i] = "Index_" + finger_part_names[i];
      index_state.position[i] = q_fingers[1][i];
      index_state.velocity[i] = dq_fingers[1][i];
      index_state.effort[i] = tauFing[1][i];

      middle_state.name[i] = "Middle_" + finger_part_names[i];
      middle_state.position[i] = q_fingers[2][i];
      middle_state.velocity[i] = dq_fingers[2][i];
      middle_state.effort[i] = tauFing[2][i];

      ring_state.name[i] = "Ring_" + finger_part_names[i];
      ring_state.position[i] = q_fingers[3][i];
      ring_state.velocity[i] = dq_fingers[3][i];
      ring_state.effort[i] = tauFing[3][i];

      little_state.name[i] = "LIttle_" + finger_part_names[i];
      little_state.position[i] = q_fingers[4][i];
      little_state.velocity[i] = dq_fingers[5][i];
      little_state.effort[i] = tauFing[5][i];
    }
    pub_index_state.publish(index_state);
    pub_middle_state.publish(middle_state);
    pub_ring_state.publish(ring_state);
    pub_little_state.publish(little_state);
  }
}

// Publish information relative to the synergy motor
void SoftHandPlugin::PublishMotor()
{
  mot_state.position[0] = mot.pos;
  mot_state.velocity[0] = mot.vel;
  mot_state.effort[0] = mot.effort;
  pub_mot_state.publish(mot_state);
}

// Initialize useful parameters
void SoftHandPlugin::InitParams(sdf::ElementPtr _sdf)
{
  // Retrieve values from tags (if inserted)
  INITIALIZE_PARAMETER_FROM_TAG( double, spring_k, _sdf, "spring_k", 10 ) // fingers spring rate [N m /rad]

  // Retrieve PID gains of the finger controllers (used ONLY in fully actuation mode)
  INITIALIZE_PARAMETER_FROM_TAG( double, kP_fing, _sdf, "kP_fing", 10 ) 
  INITIALIZE_PARAMETER_FROM_TAG( double, kI_fing, _sdf, "kI_fing", 0 )  
  INITIALIZE_PARAMETER_FROM_TAG( double, kD_fing, _sdf, "kD_fing", 0.5 )
  // and assign them to all the finger controllers
  for (int i = 0; i < 4; ++i)
  { 
    // thumb
    if (i==3)
    {
      // do nothing!
    }else{
      pid_thumb[i].P = kP_fing;
      pid_thumb[i].I = kI_fing;
      pid_thumb[i].D = kD_fing;
    }
    // // index
    pid_index[i].P = kP_fing;
    pid_index[i].I = kI_fing;
    pid_index[i].D = kD_fing;
    // // middle
    pid_middle[i].P = kP_fing;
    pid_middle[i].I = kI_fing;
    pid_middle[i].D = kD_fing;
    // // ring
    pid_ring[i].P = kP_fing;
    pid_ring[i].I = kI_fing;
    pid_ring[i].D = kD_fing;
    // // little
    pid_little[i].P = kP_fing;
    pid_little[i].I = kI_fing;
    pid_little[i].D = kD_fing;
  }

  // Initialize joint variables
  thumb_joint.resize(3);
  index_joint.resize(4);
  middle_joint.resize(4);
  ring_joint.resize(4);
  little_joint.resize(4);

  mot_state.position.resize(1);
  mot_state.velocity.resize(1);
  mot_state.effort.resize(1);

  // Initialize fingers state
  thumb_state.name.resize(3);
  thumb_state.position.resize(3);
  thumb_state.velocity.resize(3);
  thumb_state.effort.resize(3);

  index_state.name.resize(4);
  index_state.position.resize(4);
  index_state.velocity.resize(4);
  index_state.effort.resize(4);

  middle_state.name.resize(4);
  middle_state.position.resize(4);
  middle_state.velocity.resize(4);
  middle_state.effort.resize(4);

  ring_state.name.resize(4);
  ring_state.position.resize(4);
  ring_state.velocity.resize(4);
  ring_state.effort.resize(4);

  little_state.name.resize(4);
  little_state.position.resize(4);
  little_state.velocity.resize(4);
  little_state.effort.resize(4);

  // Initialize fingers command for fully actiation modes
  cmd_thumb.data.resize(3);
  cmd_index.data.resize(4);
  cmd_middle.data.resize(4);
  cmd_ring.data.resize(4);
  cmd_little.data.resize(4);  
}

// Compose topics' names according to the type of actuators
void SoftHandPlugin::topicNames(std::string ns_name, std::string joint_name)
{
  // Compose string name for the publishers 
  pub_thumb_name = ns_name + "/thumb_state";
  pub_index_name = ns_name + "/index_state";
  pub_middle_name = ns_name + "/middle_state";
  pub_ring_name = ns_name + "/ring_state";
  pub_little_name = ns_name + "/little_state";

  // Compose string name for the state publisher
  link_pub_name = ns_name + "/fingers_state";

  switch(operationMode){
    case(SoftSynergies):
        cmd_ref_name = ns_name + "/synergy_command";
        break;
    case(AdaptiveSynergiesTorque):
        cmd_ref_name = ns_name + "/synergy_tau_command";
        break;
    case(AdaptiveSynergiesPosition):
        cmd_ref_name = ns_name + "/synergy_pos_command";
        break;
    case(FingerTorques):
    case(FingerPositions):
        cmd_thumb_name = ns_name + "/thumb_command";
        cmd_index_name = ns_name + "/index_command";
        cmd_middle_name = ns_name + "/middle_command";
        cmd_ring_name = ns_name + "/ring_command";
        cmd_little_name = ns_name + "/little_command";
        break;
  }
}

// Main load function of the plugin
void SoftHandPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  int argc = 0;
  char **argv;
  ros::init(argc, argv, "SoftHand_Plugin");

  model = _parent;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // // Retrieve namespace and control mode from a configuration file
  // ros::param::get("namespace", ns_name);
  // ros::param::get("T_sample", T_sample);

  // // Retrieve joint identifier and control mode from urdf tags
  // //act_type =_sdf->GetElement("actuator_type")->Get<string>();
  // flag_pub_el_tau =_sdf->GetElement("pub_eltau")->Get<bool>();
  // flag_pub_state =_sdf->GetElement("pub_state")->Get<bool>();
  // flag_sub_ext_tau =_sdf->GetElement("sub_ext_tau")->Get<bool>();

  // Retrieve operation mode and robot namespace from urdf tags
  operationMode = (OperationModes) _sdf->GetElement("operation_mode")->Get<int>();
  INITIALIZE_PARAMETER_FROM_TAG( std::string, ns_name, _sdf, "namespace", "toBEAssigned" ) // fingers spring rate [N m /rad]

  // Everything-is-fine message
  std::string ok_msg = "SoftHandPlugin on " + ns_name + " started with " + mode_names[operationMode] + " control, mode n. [" + std::to_string(operationMode) + "]!";
  ROS_WARN_STREAM(ok_msg);

  // // Initialize motors, controllers and other parameters
  InitParams(_sdf);

  // Retrieve thumb joint
  int thumb_idx[3] = {0, 1, 3};
  for (int i = 0; i < 3; ++i)
  {
    std::string temp_name = ns_name + "_" + finger_names[0] + "_" + finger_part_names[thumb_idx[i]] + "_joint";
    std::cout << temp_name << std::endl;
    thumb_joint[i] = model->GetJoint(temp_name);
  }
  // Retrieve index joint
  for (int i = 0; i < 4; ++i)
  {
    std::string temp_name = ns_name + "_" + finger_names[1] + "_" + finger_part_names[i] + "_joint";
    std::cout << temp_name << std::endl;
    index_joint[i] = model->GetJoint(temp_name);
  }
  // Retrieve middle joint
  for (int i = 0; i < 4; ++i)
  {
    std::string temp_name = ns_name + "_" + finger_names[2] + "_" + finger_part_names[i] + "_joint";
    std::cout << temp_name << std::endl;
    middle_joint[i] = model->GetJoint(temp_name);
  }
  // Retrieve ring joint
  for (int i = 0; i < 4; ++i)
  {
    std::string temp_name = ns_name + "_" + finger_names[3] + "_" + finger_part_names[i] + "_joint";
    std::cout << temp_name << std::endl;
    ring_joint[i] = model->GetJoint(temp_name);
  }
  // Retrieve little joint
  for (int i = 0; i < 4; ++i)
  {
    std::string temp_name = ns_name + "_" + finger_names[4] + "_" + finger_part_names[i] + "_joint";
    std::cout << temp_name << std::endl;
    little_joint[i] = model->GetJoint(temp_name);
  }

  // Compose the topic names
  SoftHandPlugin::topicNames(ns_name, joint_name);

  // Subscribers
  if ( (operationMode == SoftSynergies) ||  (operationMode == AdaptiveSynergiesTorque) ||  (operationMode == AdaptiveSynergiesPosition))
  {
    sub_cmd = n.subscribe(cmd_ref_name, 10, &SoftHandPlugin::getRef_callback, this);
  }

  if ( (operationMode == FingerTorques) ||  (operationMode == FingerPositions))
  {
    sub_thumb_cmd = n.subscribe(cmd_thumb_name, 10, &SoftHandPlugin::getThumbCmd_callback, this);
    sub_index_cmd = n.subscribe(cmd_index_name, 10, &SoftHandPlugin::getIndexCmd_callback, this);
    sub_middle_cmd = n.subscribe(cmd_middle_name, 10, &SoftHandPlugin::getMiddleCmd_callback, this);
    sub_ring_cmd = n.subscribe(cmd_ring_name, 10, &SoftHandPlugin::getRingCmd_callback, this);
    sub_little_cmd = n.subscribe(cmd_little_name, 10, &SoftHandPlugin::getLittleCmd_callback, this);
  }

  // // Publishers 
  if ( (operationMode == AdaptiveSynergiesPosition) || (operationMode == AdaptiveSynergiesTorque))
  {
      std::string motor_state_name = ns_name + "/motor_state";
      pub_mot_state = n.advertise<sensor_msgs::JointState>(motor_state_name, 500);
  }

  if (flag_pub_state)
  {
    // pubL_state = n.advertise<softhand_plugin::state_info>(link_pub_name, 500);
    pub_thumb_state = n.advertise<sensor_msgs::JointState>(pub_thumb_name, 500);
    pub_index_state = n.advertise<sensor_msgs::JointState>(pub_index_name, 500);
    pub_middle_state = n.advertise<sensor_msgs::JointState>(pub_middle_name, 500);
    pub_ring_state = n.advertise<sensor_msgs::JointState>(pub_ring_name, 500);
    pub_little_state = n.advertise<sensor_msgs::JointState>(pub_little_name, 500);
  }

  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SoftHandPlugin::OnUpdate, this, _1));

}





// std::vector<std::vector<int>> pippo;

// pippo.resize(5);
// pippo[0].resize(3);

// //for (int i = 1; i < 5; i++)
// for( auto i = pippo.begin(); i < pippo.end(); i++)
// {
//   // i->begin()  === (*i).begin()
//   for( auto j = i->begin(); j < i->end(); j++)
//     //*j = 5;  
//     (*j)->getPosition()
//   //fai cose
// }