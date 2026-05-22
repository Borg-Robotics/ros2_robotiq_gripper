// Copyright (c) 2014 Konstantinos Chatzilygeroudis
// Copyright (c) 2025 Borg Robotics (ROS2 / Gazebo Classic 11 port)
//
// BSD 3-Clause License — see original:
// https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins
//
// Minimal, ROS-free Gazebo Classic model plugin that enforces a mimic-joint
// constraint every physics step.  Attach one <plugin> per mimic joint.
//
// SDF parameters (inside <plugin>):
//   <joint>        — name of the actuated (leader) joint
//   <mimicJoint>   — name of the mimic (follower) joint
//   <multiplier>   — position multiplier  (default 1.0)
//   <offset>       — position offset      (default 0.0)
//   <maxEffort>    — effort clamp for PID  (default: joint URDF limit)
//   <hasPID>       — if present, use PID force control instead of SetPosition
//     sub-elements: <p>, <i>, <d> gains
//   <sensitiveness> — dead-band threshold  (default 0.0)

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{

class MimicJointPlugin : public ModelPlugin
{
public:
  MimicJointPlugin() = default;
  ~MimicJointPlugin() override = default;

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();

    // --- Required parameters ---------------------------------------------------
    if (!sdf->HasElement("joint") || !sdf->HasElement("mimicJoint")) {
      gzerr << "MimicJointPlugin: <joint> and <mimicJoint> are required.\n";
      return;
    }
    const auto joint_name = sdf->Get<std::string>("joint");
    const auto mimic_name = sdf->Get<std::string>("mimicJoint");

    joint_ = model_->GetJoint(joint_name);
    mimic_joint_ = model_->GetJoint(mimic_name);
    if (!joint_ || !mimic_joint_) {
      gzerr << "MimicJointPlugin: joint '" << joint_name
             << "' or mimicJoint '" << mimic_name << "' not found in model.\n";
      return;
    }

    // --- Optional parameters ---------------------------------------------------
    multiplier_ = sdf->HasElement("multiplier") ? sdf->Get<double>("multiplier") : 1.0;
    offset_     = sdf->HasElement("offset")     ? sdf->Get<double>("offset")     : 0.0;
    sensitiveness_ = sdf->HasElement("sensitiveness") ? sdf->Get<double>("sensitiveness") : 0.0;
    max_effort_ = sdf->HasElement("maxEffort")  ? sdf->Get<double>("maxEffort")
                                                 : mimic_joint_->GetEffortLimit(0);

    // --- PID mode (optional) ---------------------------------------------------
    has_pid_ = sdf->HasElement("hasPID");
    if (has_pid_) {
      double p = 0.0, i = 0.0, d = 0.0;
      auto pid_elem = sdf->GetElement("hasPID");
      if (pid_elem->HasElement("p")) p = pid_elem->Get<double>("p");
      if (pid_elem->HasElement("i")) i = pid_elem->Get<double>("i");
      if (pid_elem->HasElement("d")) d = pid_elem->Get<double>("d");
      pid_ = common::PID(p, i, d, max_effort_, -max_effort_);
    } else {
      // Direct position mode — set max force so SetPosition can move the joint.
      mimic_joint_->SetParam("fmax", 0, max_effort_);
    }

    // --- Connect to world update -----------------------------------------------
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MimicJointPlugin::OnUpdate, this));

    gzmsg << "MimicJointPlugin loaded: " << joint_name
          << " -> " << mimic_name
          << "  mult=" << multiplier_ << "  off=" << offset_
          << "  pid=" << (has_pid_ ? "yes" : "no") << "\n";
  }

private:
  void OnUpdate()
  {
    const double target = joint_->Position(0) * multiplier_ + offset_;
    const double current = mimic_joint_->Position(0);

    if (std::fabs(target - current) < sensitiveness_)
      return;

    if (has_pid_) {
      const double dt = world_->Physics()->GetMaxStepSize();
      const double error = target - current;
      const double effort = ignition::math::clamp(
          pid_.Update(error, common::Time(dt)),
          -max_effort_, max_effort_);
      mimic_joint_->SetForce(0, effort);
    } else {
      // Gazebo >= 9: third arg = true preserves link velocity (gravity-safe).
      mimic_joint_->SetPosition(0, target, true);
    }
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr joint_;
  physics::JointPtr mimic_joint_;

  double multiplier_{1.0};
  double offset_{0.0};
  double sensitiveness_{0.0};
  double max_effort_{1000.0};

  bool has_pid_{false};
  common::PID pid_;

  event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)

}  // namespace gazebo
