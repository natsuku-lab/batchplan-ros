// A MoveIt planner that plans with batchplan: each request, with its planning scene, goes to the
// batchplan_planner node's MoveGroup action, and the timed trajectory it returns is the plan.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/get_planner_params.hpp>
#include <moveit_msgs/srv/set_planner_params.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace batchplan_moveit
{
using MoveGroup = moveit_msgs::action::MoveGroup;
using moveit_msgs::msg::MoveItErrorCodes;
using Parameters = std::map<std::string, std::string>;
using GetPlannerParams = moveit_msgs::srv::GetPlannerParams;
using SetPlannerParams = moveit_msgs::srv::SetPlannerParams;
using Clock = std::chrono::steady_clock;

// One request's cancellation: whether it was asked for, and the goal to cancel once the node has
// accepted it.
struct Cancellation
{
  std::atomic_bool requested{ false };
  std::mutex mutex;
  bool cancel_on_accept{ false };
  rclcpp_action::ClientGoalHandle<MoveGroup>::SharedPtr goal;
};

// The action client, on a node and executor of its own: waiting for an answer never needs
// move_group's executor, which is busy calling the planner.
class Client
{
public:
  explicit Client(const std::string& action)
  {
    node_ = std::make_shared<rclcpp::Node>("batchplan_moveit_client",
                                           rclcpp::NodeOptions().use_global_arguments(false));
    client_ = rclcpp_action::create_client<MoveGroup>(node_, action);
    const auto slash = action.find_last_of('/');
    const auto prefix = slash == std::string::npos ? std::string() : action.substr(0, slash + 1);
    get_params_ = node_->create_client<GetPlannerParams>(prefix + "get_planner_params");
    set_params_ = node_->create_client<SetPlannerParams>(prefix + "set_planner_params");
    executor_.add_node(node_);
    spinner_ = std::thread([this] { executor_.spin(); });
  }

  ~Client()
  {
    executor_.cancel();
    spinner_.join();
  }

  // The planner's answer by `deadline`, or an error code saying why there is none.
  MoveGroup::Result plan(const MoveGroup::Goal& goal, Clock::time_point deadline,
                         const std::shared_ptr<Cancellation>& cancellation)
  {
    if (!client_->wait_for_action_server(std::max(Clock::duration::zero(), deadline - Clock::now())))
      return failure(MoveItErrorCodes::COMMUNICATION_FAILURE, "batchplan_planner is not running");
    auto configured = goal;
    configured.request.allowed_planning_time = std::chrono::duration<double>(deadline - Clock::now()).count();
    if (configured.request.allowed_planning_time <= 0.0)
      return failure(MoveItErrorCodes::TIMED_OUT, "planning time exhausted before sending request");
    rclcpp_action::Client<MoveGroup>::SendGoalOptions options;
    options.goal_response_callback =
        [client = std::weak_ptr<rclcpp_action::Client<MoveGroup>>(client_), cancellation](auto handle) {
          std::lock_guard<std::mutex> lock(cancellation->mutex);
          cancellation->goal = handle;
          // Acceptance may arrive after terminate() or even after this plan timed out.
          if (handle && cancellation->cancel_on_accept)
            if (auto active = client.lock())
              active->async_cancel_goal(handle);
        };
    std::unique_lock<std::mutex> cancellation_lock(cancellation->mutex);
    if (cancellation->requested)
      return failure(MoveItErrorCodes::PREEMPTED, "planning cancelled");
    auto sent = client_->async_send_goal(configured, options);
    cancellation_lock.unlock();
    if (sent.wait_until(deadline) != std::future_status::ready)
    {
      cancel(cancellation);
      return failure(MoveItErrorCodes::TIMED_OUT, "batchplan_planner did not take the request in time");
    }
    auto handle = sent.get();
    if (!handle)
      return failure(MoveItErrorCodes::FAILURE, "batchplan_planner refused the request");
    auto answer = client_->async_get_result(handle);
    const bool answered = answer.wait_until(deadline + std::chrono::milliseconds(500)) == std::future_status::ready;
    {
      std::lock_guard<std::mutex> lock(cancellation->mutex);
      cancellation->goal.reset();
    }
    if (!answered)
    {
      client_->async_cancel_goal(handle);
      return failure(MoveItErrorCodes::TIMED_OUT, "no answer within the planning time");
    }
    const auto wrapped = answer.get();
    if (!wrapped.result)
      return failure(MoveItErrorCodes::FAILURE, "batchplan_planner returned no result");
    return *wrapped.result;
  }

  void cancel(const std::shared_ptr<Cancellation>& cancellation)
  {
    std::lock_guard<std::mutex> lock(cancellation->mutex);
    cancellation->cancel_on_accept = true;
    if (cancellation->goal)
      client_->async_cancel_goal(cancellation->goal);
  }

  std::optional<Parameters> parameters(const std::string& group, const std::string& planner,
                                       Clock::time_point deadline)
  {
    if (Clock::now() >= deadline ||
        !get_params_->wait_for_service(std::max(Clock::duration::zero(), deadline - Clock::now())))
      return std::nullopt;
    auto request = std::make_shared<GetPlannerParams::Request>();
    request->group = group;
    request->planner_config = planner;
    auto pending = get_params_->async_send_request(request);
    if (pending.wait_until(deadline) != std::future_status::ready)
    {
      get_params_->remove_pending_request(pending);
      return std::nullopt;
    }
    const auto reply = pending.get();
    if (reply->params.keys.empty() || reply->params.keys.size() != reply->params.values.size())
      return std::nullopt;
    Parameters values;
    for (std::size_t k = 0; k < reply->params.keys.size(); ++k)
      values[reply->params.keys[k]] = reply->params.values[k];
    return values;
  }

  // Replaces a group's settings for a planner with the node's defaults and `values`. Returns the
  // settings the node then holds, which are the ones before if it refused a value (the service
  // has no status to say so), or nothing if it did not answer by `deadline`.
  std::optional<Parameters> set(const std::string& group, const std::string& planner, const Parameters& values,
                                Clock::time_point deadline)
  {
    if (!set_params_->wait_for_service(std::max(Clock::duration::zero(), deadline - Clock::now())))
      return std::nullopt;
    auto request = std::make_shared<SetPlannerParams::Request>();
    request->group = group;
    request->planner_config = planner;
    request->replace = true;
    for (const auto& [key, value] : values)
    {
      request->params.keys.push_back(key);
      request->params.values.push_back(value);
    }
    auto pending = set_params_->async_send_request(request);
    if (pending.wait_until(deadline) != std::future_status::ready)
    {
      set_params_->remove_pending_request(pending);
      return std::nullopt;
    }
    return parameters(group, planner, deadline);
  }

private:
  static MoveGroup::Result failure(int code, const std::string& message)
  {
    MoveGroup::Result result;
    result.error_code.val = code;
    result.error_code.message = message;
    result.error_code.source = "batchplan_moveit";
    return result;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<MoveGroup>::SharedPtr client_;
  rclcpp::Client<GetPlannerParams>::SharedPtr get_params_;
  rclcpp::Client<SetPlannerParams>::SharedPtr set_params_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
};

class BatchplanPlanningContext : public planning_interface::PlanningContext
{
public:
  // `default_planning_time` is the node's, for requests that allow none.
  BatchplanPlanningContext(const std::string& group, moveit::core::RobotModelConstPtr model,
                           std::shared_ptr<Client> client, double default_planning_time)
    : planning_interface::PlanningContext("batchplan", group)
    , model_(std::move(model))
    , client_(std::move(client))
    , default_planning_time_(default_planning_time)
  {
  }

  void solve(planning_interface::MotionPlanResponse& res) override
  {
    res.trajectory.reset();
    std::shared_ptr<Cancellation> cancellation;
    {
      std::lock_guard<std::mutex> lock(cancellation_mutex_);
      if (!cancellation_->requested)
        cancellation_ = std::make_shared<Cancellation>();
      cancellation = cancellation_;
    }
    const auto begun = std::chrono::steady_clock::now();
    const auto& request = getMotionPlanRequest();
    MoveGroup::Goal goal;
    goal.request = request;
    // The start is the request's, filled in from the scene's current state.
    moveit::core::RobotState start = getPlanningScene()->getCurrentState();
    moveit::core::robotStateMsgToRobotState(request.start_state, start);
    moveit::core::robotStateToRobotStateMsg(start, goal.request.start_state);
    getPlanningScene()->getPlanningSceneMsg(goal.planning_options.planning_scene_diff);
    goal.planning_options.plan_only = true;

    const double seconds = request.allowed_planning_time > 0.0 ? request.allowed_planning_time : default_planning_time_;
    res.planner_id = "batchplan";
    moveit::core::robotStateToRobotStateMsg(start, res.start_state);
    res.error_code.val = MoveItErrorCodes::PREEMPTED;
    res.error_code.source = "batchplan_moveit";
    res.error_code.message = "planning cancelled";
    if (!std::isfinite(seconds) ||
        seconds > std::chrono::duration<double>(Clock::duration::max() - begun.time_since_epoch()).count() - 1.0)
    {
      res.error_code.val = MoveItErrorCodes::INVALID_MOTION_PLAN;
      res.error_code.message = "the planning time must be finite and representable";
      return;
    }
    const auto deadline = begun + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    if (!cancellation->requested)
    {
      const auto result = client_->plan(goal, deadline, cancellation);
      res.error_code = result.error_code;
      if (result.error_code.val == MoveItErrorCodes::SUCCESS)
      {
        auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(model_, getGroupName());
        trajectory->setRobotTrajectoryMsg(start, result.planned_trajectory);
        res.trajectory = trajectory;
      }
    }
    if (cancellation->requested)
    {
      res.trajectory.reset();
      res.error_code.val = MoveItErrorCodes::PREEMPTED;
      res.error_code.message = "planning cancelled";
    }
    res.planning_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - begun).count();
  }

  void solve(planning_interface::MotionPlanDetailedResponse& res) override
  {
    planning_interface::MotionPlanResponse plan;
    solve(plan);
    res.error_code = plan.error_code;
    res.planner_id = plan.planner_id;
    if (plan.trajectory)
    {
      res.trajectory.push_back(plan.trajectory);
      res.description.push_back("plan");
      res.processing_time.push_back(plan.planning_time);
    }
  }

  bool terminate() override
  {
    std::lock_guard<std::mutex> lock(cancellation_mutex_);
    cancellation_->requested = true;
    client_->cancel(cancellation_);
    return true;
  }

  void clear() override
  {
    std::lock_guard<std::mutex> lock(cancellation_mutex_);
    cancellation_ = std::make_shared<Cancellation>();
  }

private:
  moveit::core::RobotModelConstPtr model_;
  std::shared_ptr<Client> client_;
  double default_planning_time_;
  std::mutex cancellation_mutex_;
  std::shared_ptr<Cancellation> cancellation_ = std::make_shared<Cancellation>();
};

class BatchplanPlannerManager : public planning_interface::PlannerManager
{
public:
  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override
  {
    model_ = model;
    const std::string parameter = parameter_namespace + ".action";
    if (!node->has_parameter(parameter))
      node->declare_parameter<std::string>(parameter, "/batchplan_planner/move_group");
    client_ = std::make_shared<Client>(node->get_parameter(parameter).as_string());
    logger_ = node->get_logger();
    groups_ = model->getJointModelGroupNames();
    // The node offers its services once it has loaded the robot, which can take a while the first
    // time. Without its settings MoveIt has none to show or edit; plans use the node's own.
    if (!load(std::chrono::seconds(60)))
      RCLCPP_WARN(logger_, "batchplan_planner gave no settings within 60 s; its planner parameters are not shown");
    return true;
  }

  std::string getDescription() const override
  {
    return "batchplan";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algorithms) const override
  {
    algorithms = { "batchplan" };
  }

  // MoveIt's SetPlannerParams hands over every configuration with one changed: a group's, named
  // "group[planner]", or one for every group, named "planner". Each changed one goes to the node,
  // and what the node then holds is what MoveIt shows. One for every group is applied to each
  // group's and not kept, since MoveIt would show it over the groups' own values.
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& configurations) override
  {
    if (defaults_.empty())
      load(std::chrono::seconds(5));
    auto held = config_settings_;
    for (const auto& [name, asked] : configurations)
    {
      const auto bracket = name.find('[');
      const bool every_group = bracket == std::string::npos;
      const auto planner = every_group ? name : name.substr(bracket + 1, name.size() - bracket - 2);
      for (const auto& group : every_group ? groups_ : std::vector<std::string>{ name.substr(0, bracket) })
      {
        auto& settings = held[group + "[" + planner + "]"];
        // A group's configuration is whole, over the defaults; one for every group adds to each.
        auto wanted = every_group && !settings.config.empty() ? settings.config : defaults_;
        for (const auto& [key, value] : asked.config)
          wanted[key] = value;
        if (wanted == settings.config)
          continue;
        const auto stored = client_->set(group, planner, wanted, Clock::now() + std::chrono::seconds(5));
        if (!stored)
        {
          RCLCPP_ERROR(logger_, "batchplan_planner did not take the settings of '%s' for '%s'", planner.c_str(),
                       group.c_str());
          continue;
        }
        if (*stored == settings.config)
          RCLCPP_WARN(logger_, "batchplan_planner kept the settings of '%s' for '%s'; its log names a value it refused",
                      planner.c_str(), group.c_str());
        settings.name = group + "[" + planner + "]";
        settings.group = group;
        settings.config = *stored;
      }
    }
    planning_interface::PlannerManager::setPlannerConfigurations(held);
  }

  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    // The node refuses groups and planners it does not know. Its own time for requests that
    // allow none is 5 s unless its settings say otherwise.
    double default_planning_time = 5.0;
    const auto settings = config_settings_.find(req.group_name + "[batchplan]");
    if (settings != config_settings_.end())
    {
      const auto time = settings->second.config.find("default_planning_time");
      if (time != settings->second.config.end())
        default_planning_time = std::stod(time->second);
    }
    auto context = std::make_shared<BatchplanPlanningContext>(req.group_name, model_, client_, default_planning_time);
    context->setPlanningScene(scene);
    context->setMotionPlanRequest(req);
    error_code.val = MoveItErrorCodes::SUCCESS;
    return context;
  }

  // batchplan plans to joint and pose goals and keeps path constraints on links' positions and
  // orientations, which the node checks further; not joint or visibility path constraints, nor
  // trajectory constraints.
  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override
  {
    const auto& path = req.path_constraints;
    return path.joint_constraints.empty() && path.visibility_constraints.empty() &&
           req.trajectory_constraints.constraints.empty();
  }

private:
  // The node's default settings as each group's configuration, if the node answers within `wait`.
  bool load(Clock::duration wait)
  {
    const auto defaults = client_->parameters("", "batchplan", Clock::now() + wait);
    if (!defaults)
      return false;
    defaults_ = *defaults;
    for (const auto& group : groups_)
    {
      auto& settings = config_settings_[group + "[batchplan]"];
      settings.name = group + "[batchplan]";
      settings.group = group;
      settings.config = defaults_;
    }
    return true;
  }

  moveit::core::RobotModelConstPtr model_;
  std::shared_ptr<Client> client_;
  rclcpp::Logger logger_ = rclcpp::get_logger("batchplan_moveit");
  std::vector<std::string> groups_;
  Parameters defaults_;
};
}  // namespace batchplan_moveit

PLUGINLIB_EXPORT_CLASS(batchplan_moveit::BatchplanPlannerManager, planning_interface::PlannerManager)
