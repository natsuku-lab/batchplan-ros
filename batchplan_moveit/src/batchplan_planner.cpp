// A MoveIt planner that plans with batchplan: each request, with its planning scene, goes to the
// batchplan_planner node's MoveGroup action, and the timed trajectory it returns is the plan.

#include <chrono>
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
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace batchplan_moveit
{
using MoveGroup = moveit_msgs::action::MoveGroup;
using moveit_msgs::msg::MoveItErrorCodes;

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
    executor_.add_node(node_);
    spinner_ = std::thread([this] { executor_.spin(); });
  }

  ~Client()
  {
    executor_.cancel();
    spinner_.join();
  }

  // The planner's answer, or an error code saying why there is none, within `timeout`.
  MoveGroup::Result plan(const MoveGroup::Goal& goal, std::chrono::nanoseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    if (!client_->wait_for_action_server(std::chrono::seconds(1)))
      return failure(MoveItErrorCodes::COMMUNICATION_FAILURE, "batchplan_planner is not running");
    auto sent = client_->async_send_goal(goal);
    if (sent.wait_until(deadline) != std::future_status::ready)
      return failure(MoveItErrorCodes::TIMED_OUT, "batchplan_planner did not take the request in time");
    auto handle = sent.get();
    if (!handle)
      return failure(MoveItErrorCodes::FAILURE, "batchplan_planner refused the request");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_ = handle;
    }
    auto answer = client_->async_get_result(handle);
    const bool answered = answer.wait_until(deadline) == std::future_status::ready;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_.reset();
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

  void cancel()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_)
      client_->async_cancel_goal(current_);
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
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  std::mutex mutex_;
  rclcpp_action::ClientGoalHandle<MoveGroup>::SharedPtr current_;
};

class BatchplanPlanningContext : public planning_interface::PlanningContext
{
public:
  BatchplanPlanningContext(const std::string& group, moveit::core::RobotModelConstPtr model,
                           std::shared_ptr<Client> client)
    : planning_interface::PlanningContext("batchplan", group), model_(std::move(model)), client_(std::move(client))
  {
  }

  void solve(planning_interface::MotionPlanResponse& res) override
  {
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

    const double seconds = request.allowed_planning_time > 0.0 ? request.allowed_planning_time : 5.0;
    // The answer may take a little longer than the planning itself.
    const auto timeout = std::chrono::duration<double>(seconds + 0.5);
    const auto result = client_->plan(goal, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));

    res.planner_id = "batchplan";
    res.error_code = result.error_code;
    moveit::core::robotStateToRobotStateMsg(start, res.start_state);
    if (result.error_code.val == MoveItErrorCodes::SUCCESS)
    {
      auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(model_, getGroupName());
      trajectory->setRobotTrajectoryMsg(start, result.planned_trajectory);
      res.trajectory = trajectory;
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
    client_->cancel();
    return true;
  }

  void clear() override
  {
  }

private:
  moveit::core::RobotModelConstPtr model_;
  std::shared_ptr<Client> client_;
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

  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    auto context = std::make_shared<BatchplanPlanningContext>(req.group_name, model_, client_);
    context->setPlanningScene(scene);
    context->setMotionPlanRequest(req);
    error_code.val = MoveItErrorCodes::SUCCESS;
    return context;
  }

  // batchplan plans to joint and pose goals; it does not follow path or trajectory constraints.
  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override
  {
    const auto& path = req.path_constraints;
    return path.joint_constraints.empty() && path.position_constraints.empty() &&
           path.orientation_constraints.empty() && path.visibility_constraints.empty() &&
           req.trajectory_constraints.constraints.empty();
  }

private:
  moveit::core::RobotModelConstPtr model_;
  std::shared_ptr<Client> client_;
};
}  // namespace batchplan_moveit

PLUGINLIB_EXPORT_CLASS(batchplan_moveit::BatchplanPlannerManager, planning_interface::PlannerManager)
