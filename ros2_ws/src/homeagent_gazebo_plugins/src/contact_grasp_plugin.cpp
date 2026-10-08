#include <algorithm>
#include <functional>
#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class HomeAgentContactGraspPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = std::move(model);
    this->world_ = this->model_->GetWorld();

    this->target_model_name_ = GetString(sdf, "target_model", "physical_cup");
    this->target_link_name_ = GetString(sdf, "target_link", "link");
    this->attach_link_name_ = GetString(sdf, "attach_link", "tool_link");
    this->fixed_finger_token_ =
      GetString(sdf, "fixed_finger_token", "gripper_fixed_finger_link");
    this->moving_finger_token_ =
      GetString(sdf, "moving_finger_token", "gripper_finger_link");
    this->gripper_joint_name_ = GetString(sdf, "gripper_joint", "gripper_joint");
    this->close_threshold_ = GetDouble(sdf, "close_threshold", 0.008);
    this->open_threshold_ = GetDouble(sdf, "open_threshold", 0.020);
    this->contact_window_sec_ = GetDouble(sdf, "contact_window_sec", 0.20);
    this->enable_gravity_on_attach_ =
      sdf->HasElement("enable_gravity_on_attach") &&
      sdf->Get<bool>("enable_gravity_on_attach");

    auto * contact_manager = this->world_->Physics()->GetContactManager();
    if (contact_manager)
      contact_manager->SetNeverDropContacts(true);

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&HomeAgentContactGraspPlugin::OnUpdate, this));

    gzmsg << "[HomeAgentContactGrasp] ready model=" << this->model_->GetName()
          << " target=" << this->target_model_name_
          << " attach_link=" << this->attach_link_name_
          << " gravity_on_attach=" << this->enable_gravity_on_attach_ << "\n";
  }

private:
  static std::string GetString(
    const sdf::ElementPtr & sdf,
    const std::string & key,
    const std::string & fallback)
  {
    return sdf->HasElement(key) ? sdf->Get<std::string>(key) : fallback;
  }

  static double GetDouble(
    const sdf::ElementPtr & sdf,
    const std::string & key,
    double fallback)
  {
    return sdf->HasElement(key) ? sdf->Get<double>(key) : fallback;
  }

  bool IsTargetCollision(const std::string & scoped_name) const
  {
    return scoped_name.find(this->target_model_name_ + "::") != std::string::npos;
  }

  bool IsFingerTargetPair(
    const std::string & first,
    const std::string & second,
    const std::string & finger_token) const
  {
    return (
      first.find(finger_token) != std::string::npos && IsTargetCollision(second)) ||
      (second.find(finger_token) != std::string::npos && IsTargetCollision(first));
  }

  void OnUpdate()
  {
    if (!this->world_ || !this->model_)
      return;

    const common::Time now = this->world_->SimTime();
    auto * contact_manager = this->world_->Physics()->GetContactManager();
    if (contact_manager)
    {
      const unsigned int count = contact_manager->GetContactCount();
      for (unsigned int i = 0; i < count; ++i)
      {
        physics::Contact * contact = contact_manager->GetContact(i);
        if (!contact || contact->count <= 0 ||
            !contact->collision1 || !contact->collision2)
          continue;

        const std::string first = contact->collision1->GetScopedName();
        const std::string second = contact->collision2->GetScopedName();

        if (IsFingerTargetPair(first, second, this->fixed_finger_token_))
          this->last_fixed_contact_ = now;
        if (IsFingerTargetPair(first, second, this->moving_finger_token_))
          this->last_moving_contact_ = now;
      }
    }

    physics::JointPtr gripper_joint = this->model_->GetJoint(this->gripper_joint_name_);
    if (!gripper_joint)
      return;

    const double gripper_position = gripper_joint->Position(0);

    if (!this->attached_)
    {
      const bool fixed_recent =
        (now - this->last_fixed_contact_).Double() >= 0.0 &&
        (now - this->last_fixed_contact_).Double() <= this->contact_window_sec_;
      const bool moving_recent =
        (now - this->last_moving_contact_).Double() >= 0.0 &&
        (now - this->last_moving_contact_).Double() <= this->contact_window_sec_;

      if (gripper_position <= this->close_threshold_ && fixed_recent && moving_recent)
        AttachTarget();
    }
    else if (gripper_position >= this->open_threshold_)
    {
      DetachTarget();
    }
  }

  void AttachTarget()
  {
    physics::ModelPtr target_model = this->world_->ModelByName(this->target_model_name_);
    physics::LinkPtr parent_link = this->model_->GetLink(this->attach_link_name_);
    physics::LinkPtr child_link =
      target_model ? target_model->GetLink(this->target_link_name_) : physics::LinkPtr();

    if (!target_model || !parent_link || !child_link)
      return;

    this->grasp_joint_ = this->model_->CreateJoint(
      this->grasp_joint_name_, "fixed", parent_link, child_link);
    if (!this->grasp_joint_)
      return;

    this->grasp_joint_->Init();
    if (this->enable_gravity_on_attach_)
    {
      child_link->SetGravityMode(true);
      gzmsg << "[HomeAgentContactGrasp] gravity enabled on attached target\n";
    }
    this->attached_ = true;

    gzmsg << "[HomeAgentContactGrasp] ATTACHED target="
          << this->target_model_name_ << " after bilateral contact\n";
  }

  void DetachTarget()
  {
    if (this->grasp_joint_)
      this->grasp_joint_->Detach();

    this->model_->RemoveJoint(this->grasp_joint_name_);
    this->grasp_joint_.reset();
    this->attached_ = false;

    gzmsg << "[HomeAgentContactGrasp] DETACHED target="
          << this->target_model_name_ << "\n";
  }

private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
  physics::JointPtr grasp_joint_;

  std::string target_model_name_;
  std::string target_link_name_;
  std::string attach_link_name_;
  std::string fixed_finger_token_;
  std::string moving_finger_token_;
  std::string gripper_joint_name_;
  const std::string grasp_joint_name_{"homeagent_contact_grasp_joint"};

  double close_threshold_{0.008};
  double open_threshold_{0.020};
  double contact_window_sec_{0.20};
  bool attached_{false};
  bool enable_gravity_on_attach_{false};
  common::Time last_fixed_contact_;
  common::Time last_moving_contact_;
};

GZ_REGISTER_MODEL_PLUGIN(HomeAgentContactGraspPlugin)
}  // namespace gazebo
