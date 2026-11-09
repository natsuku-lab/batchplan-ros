// A MoveIt planner that plans with batchplan: each request, with its planning scene, goes to the
// batchplan_planner node's MoveGroup action, and the timed trajectory it returns is the plan.

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

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

  // The shared deadline includes waiting for another context and applying its settings.
  MoveGroup::Result plan(const MoveGroup::Goal& goal, const Parameters& parameters,
                        Clock::time_point deadline, const std::shared_ptr<Cancellation>& cancellation)
  {
    // Applying a configuration and planning with it must not interleave with another context.
    std::unique_lock<std::timed_mutex> planning_lock(planning_mutex_, std::defer_lock);
    while (!cancellation->requested)
    {
      if (planning_lock.try_lock_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(10))))
        break;
      if (Clock::now() >= deadline)
        return failure(MoveItErrorCodes::TIMED_OUT, "planning client is busy");
    }
    if (cancellation->requested)
      return failure(MoveItErrorCodes::PREEMPTED, "planning cancelled");
    const auto configure_error = configure(goal.request, parameters, deadline);
    if (cancellation->requested)
      return failure(MoveItErrorCodes::PREEMPTED, "planning cancelled");
    if (!configure_error.empty())
      return failure(Clock::now() >= deadline ? MoveItErrorCodes::TIMED_OUT :
                                               MoveItErrorCodes::INVALID_MOTION_PLAN, configure_error);
    if (Clock::now() >= deadline)
      return failure(MoveItErrorCodes::TIMED_OUT, "planning time exhausted while applying settings");
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

private:
  std::string configure(const moveit_msgs::msg::MotionPlanRequest& motion, const Parameters& values,
                        Clock::time_point deadline)
  {
    if (Clock::now() >= deadline ||
        !set_params_->wait_for_service(std::max(Clock::duration::zero(), deadline - Clock::now())))
      return "batchplan parameter service is not available";
    auto request = std::make_shared<SetPlannerParams::Request>();
    request->group = motion.group_name;
    request->planner_config = motion.planner_id.empty() ? "batchplan" : motion.planner_id;
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
      return "batchplan parameter update timed out";
    }
    pending.get();
    // SetPlannerParams has no status field. Read back the validated values, never assume success.
    const auto actual = parameters(request->group, request->planner_config, deadline);
    if (!actual)
      return "cannot verify batchplan parameter update";
    for (const auto& [key, value] : values)
    {
      const auto found = actual->find(key);
      if (found == actual->end() || !equivalent(key, value, found->second))
        return "batchplan rejected parameter " + key + "=" + value + "; see backend log";
    }
    return {};
  }

  static bool equivalent(const std::string& key, const std::string& requested, const std::string& actual)
  {
    if (requested == actual)
      return true;
    const bool integer = key == "solutions_per_pose" || key == "planning.control_points" ||
                         key == "planning.samples_per_span" || key == "planning.history" ||
                         key == "planning.validate_substeps" || key == "planning.shortcut.passes" ||
                         key.find(".seeds") != std::string::npos ||
                         key.find(".iterations") != std::string::npos ||
                         key.find(".max_iterations") != std::string::npos ||
                         key.find(".rng_seed") != std::string::npos;
    if (integer)
    {
      std::uint64_t a, b;
      const auto pa = std::from_chars(requested.data(), requested.data() + requested.size(), a);
      const auto pb = std::from_chars(actual.data(), actual.data() + actual.size(), b);
      return pa.ec == std::errc() && pa.ptr == requested.data() + requested.size() &&
             pb.ec == std::errc() && pb.ptr == actual.data() + actual.size() && a == b;
    }
    char *end_requested, *end_actual;
    const auto a = std::strtold(requested.c_str(), &end_requested);
    const auto b = std::strtold(actual.c_str(), &end_actual);
    return end_requested != requested.c_str() && *end_requested == '\0' &&
           end_actual != actual.c_str() && *end_actual == '\0' &&
           std::isfinite(a) && std::isfinite(b) &&
           (key == "default_planning_time" ? static_cast<double>(a) == static_cast<double>(b) :
                                            static_cast<float>(a) == static_cast<float>(b));
  }

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
  std::timed_mutex planning_mutex_;
};

class BatchplanPlanningContext : public planning_interface::PlanningContext
{
public:
  BatchplanPlanningContext(const std::string& group, moveit::core::RobotModelConstPtr model,
                           std::shared_ptr<Client> client, Parameters parameters)
    : planning_interface::PlanningContext("batchplan", group), model_(std::move(model)), client_(std::move(client)),
      parameters_(std::move(parameters))
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

    const double seconds = request.allowed_planning_time;
    res.planner_id = "batchplan";
    moveit::core::robotStateToRobotStateMsg(start, res.start_state);
    res.error_code.val = MoveItErrorCodes::PREEMPTED;
    res.error_code.source = "batchplan_moveit";
    res.error_code.message = "planning cancelled";
    if (!std::isfinite(seconds) || seconds <= 0.0 ||
        seconds > std::chrono::duration<double>(Clock::duration::max() - begun.time_since_epoch()).count() - 1.0)
    {
      res.error_code.val = MoveItErrorCodes::INVALID_MOTION_PLAN;
      res.error_code.message = "planning time must be finite, positive and representable";
      return;
    }
    const auto deadline = begun + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    if (!cancellation->requested)
    {
      // The backend owns native search attempts; MoveIt's response adapter owns final validation.
      const auto result = client_->plan(goal, parameters_, deadline, cancellation);
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
  Parameters parameters_;
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
    const auto defaults = client_->parameters("", "batchplan", Clock::now() + std::chrono::seconds(60));
    if (!defaults)
    {
      RCLCPP_ERROR(node->get_logger(), "Cannot obtain BatchPlan settings; backend must provide ~/get_planner_params");
      return false;
    }
    planning_interface::PlannerConfigurationSettings settings;
    settings.config = *defaults;
    defaults_ = *defaults;
    // MoveIt's query service inserts globals first without overwriting them with group values.
    for (const auto& group : model->getJointModelGroupNames())
    {
      settings.group = group;
      settings.name = group;
      config_settings_[settings.name] = settings;
      settings.name = group + "[batchplan]";
      config_settings_[settings.name] = settings;
    }
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

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& configurations) override
  {
    auto merged = config_settings_;
    for (const auto& [name, settings] : configurations)
    {
      auto normalized = settings;
      normalized.config = defaults_;
      for (const auto& [key, value] : settings.config)
        normalized.config[key] = value;
      merged[name] = std::move(normalized);
    }
    planning_interface::PlannerManager::setPlannerConfigurations(merged);
  }

  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    if (!req.planner_id.empty() && req.planner_id != "batchplan" && req.planner_id != "LIN")
    {
      error_code.val = MoveItErrorCodes::INVALID_MOTION_PLAN;
      error_code.message = "unknown planner '" + req.planner_id + "'";
      return {};
    }
    const auto key = req.group_name + "[" + (req.planner_id.empty() ? "batchplan" : req.planner_id) + "]";
    auto config = config_settings_.find(key);
    if (config == config_settings_.end())
      config = config_settings_.find(req.group_name);
    if (config == config_settings_.end())
    {
      error_code.val = MoveItErrorCodes::INVALID_GROUP_NAME;
      return {};
    }
    auto parameters = config->second.config;
    auto context = std::make_shared<BatchplanPlanningContext>(
        req.group_name, model_, client_, std::move(parameters));
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
  moveit::core::RobotModelConstPtr model_;
  std::shared_ptr<Client> client_;
  Parameters defaults_;
};
}  // namespace batchplan_moveit

PLUGINLIB_EXPORT_CLASS(batchplan_moveit::BatchplanPlannerManager, planning_interface::PlannerManager)
