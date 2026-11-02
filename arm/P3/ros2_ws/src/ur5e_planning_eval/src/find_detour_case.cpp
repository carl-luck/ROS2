#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <vector>

using Clock = std::chrono::steady_clock;

template<class Service>
typename Service::Response::SharedPtr query(
    const rclcpp::Node::SharedPtr& node, const std::string& name,
    const typename Service::Request::SharedPtr& request)
{
  auto client = node->create_client<Service>(name);
  if (!client->wait_for_service(std::chrono::seconds(3)))
    throw std::runtime_error("Read service unavailable: " + name);
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    throw std::runtime_error("Read service wall timeout: " + name);
  return future.get();
}

void vector_json(std::ostream& stream, const std::vector<double>& values)
{
  stream << '[';
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (i) stream << ',';
    stream << std::setprecision(17) << values[i];
  }
  stream << ']';
}

struct Check
{
  bool bounded = false;
  bool collision = false;
  bool hits_box = false;
  bool other_collision = false;
  std::string contacts;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("p3_find_detour_case");
  auto output = node->declare_parameter<std::string>("output_dir", "");
  if (output.empty() || !std::filesystem::is_directory(output))
  {
    RCLCPP_ERROR(node->get_logger(), "output_dir must be an existing unique run directory");
    rclcpp::shutdown();
    return 2;
  }
  auto path = std::filesystem::path(output);
  std::ofstream report(path / "search_report.json");
  try
  {
    const auto group_name = node->declare_parameter<std::string>("group", "ur_manipulator");
    const auto box = node->declare_parameter<std::string>("obstacle_id", "p3_box_v1");
    const auto seed = node->declare_parameter<int>("seed", 20261101);
    const auto max_candidates = node->declare_parameter<int>("max_candidates", 5000);
    const auto budget = node->declare_parameter<double>("wall_budget_s", 90.0);
    const auto step = node->declare_parameter<double>("interpolation_step_rad", 0.01);
    const auto sweeps = node->declare_parameter<std::vector<double>>("shoulder_sweep_half_angles_rad", {0.35, 0.6, 0.9, 1.2});
    const auto lower = node->declare_parameter<std::vector<double>>("candidate_lower_rad", {-2.8,-1.8,-2.0,-2.5,-2.0,-1.0});
    const auto upper = node->declare_parameter<std::vector<double>>("candidate_upper_rad", {2.8,0.3,2.0,0.5,2.0,1.0});
    if (lower.size()!=6 || upper.size()!=6 || step<=0 || !std::isfinite(step) ||
        budget<=0 || !std::isfinite(budget) || max_candidates<=0 || sweeps.empty())
      throw std::runtime_error("Invalid search parameters");
    for (std::size_t i=0; i<6; ++i)
      if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i]>=upper[i])
        throw std::runtime_error("Invalid candidate bounds");
    for (double sweep:sweeps)
      if (!std::isfinite(sweep) || sweep<=0 || sweep>1.5)
        throw std::runtime_error("Invalid sweep angle");
    auto params_req=std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    params_req->names={"robot_description", "robot_description_semantic"};
    auto params=query<rcl_interfaces::srv::GetParameters>(node,"/move_group/get_parameters",params_req);
    if (params->values.size()!=2 || params->values[0].string_value.empty() || params->values[1].string_value.empty())
      throw std::runtime_error("Missing URDF/SRDF");
    const auto urdf=params->values[0].string_value;
    const auto srdf=params->values[1].string_value;
    std::ofstream(path/"robot.urdf") << urdf;
    std::ofstream(path/"robot.srdf") << srdf;
    robot_model_loader::RobotModelLoader::Options options(urdf,srdf);
    options.load_kinematics_solvers_=false;
    robot_model_loader::RobotModelLoader loader(node,options);
    auto model=loader.getModel();
    if (!model) throw std::runtime_error("Robot model loading failed");
    auto group=model->getJointModelGroup(group_name);
    const std::vector<std::string> expected={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    if (!group || group->getVariableNames()!=expected)
      throw std::runtime_error("Search ranges require the verified six-joint order");
    auto scene_req=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    scene_req->components.components=1023;
    auto remote=query<moveit_msgs::srv::GetPlanningScene>(node,"/get_planning_scene",scene_req);
    auto scene=std::make_shared<planning_scene::PlanningScene>(model);
    if (!scene->setPlanningSceneMsg(remote->scene))
      throw std::runtime_error("Could not reconstruct scene snapshot");
    if (remote->scene.world.collision_objects.size()!=1 || remote->scene.world.collision_objects[0].id!=box ||
        !remote->scene.robot_state.attached_collision_objects.empty() || !remote->scene.world.octomap.octomap.data.empty())
      throw std::runtime_error("This module requires exactly the verified single box and no attached objects/octomap");
    rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(&remote->scene,&serialized);
    const auto& bytes=serialized.get_rcl_serialized_message();
    std::ofstream binary(path/"planning_scene.cdr",std::ios::binary);
    binary.write(reinterpret_cast<const char*>(bytes.buffer),bytes.buffer_length);
    binary.close();
    collision_detection::CollisionRequest req;
    req.group_name=group_name;
    req.contacts=true;
    req.max_contacts=100;
    req.max_contacts_per_pair=2;
    moveit::core::RobotState state(scene->getCurrentState());
    auto check=[&](const std::vector<double>& q)
    {
      Check out;
      state.setJointGroupPositions(group,q);
      state.update();
      out.bounded=state.satisfiesBounds(group);
      if (!out.bounded) return out;
      collision_detection::CollisionResult result;
      scene->checkCollision(req,result,state);
      out.collision=result.collision;
      for (const auto& pair:result.contacts)
      {
        bool hit=pair.first.first==box || pair.first.second==box;
        out.hits_box=out.hits_box || hit;
        out.other_collision=out.other_collision || !hit;
        if (!out.contacts.empty()) out.contacts+=';';
        out.contacts+=pair.first.first+"|"+pair.first.second;
      }
      return out;
    };
    std::mt19937 rng(static_cast<unsigned int>(seed));
    const auto began=Clock::now();
    int tried=0;
    bool found=false;
    std::vector<double> start,goal;
    int intervals=0;
    for (; tried<max_candidates; ++tried)
    {
      if (std::chrono::duration<double>(Clock::now()-began).count()>budget) break;
      if (tried%500==0) RCLCPP_INFO(node->get_logger(),"Offline candidate %d; no motion or scene changes",tried);
      std::vector<double> mid(6);
      for (std::size_t i=0;i<6;++i) mid[i]=std::uniform_real_distribution<double>(lower[i],upper[i])(rng);
      auto middle=check(mid);
      if (!middle.bounded || !middle.hits_box || middle.other_collision) continue;
      for (double sweep:sweeps)
      {
        auto a=mid,b=mid;
        a[0]-=sweep;b[0]+=sweep;
        auto ca=check(a),cb=check(b);
        if (!ca.bounded || !cb.bounded || ca.collision || cb.collision) continue;
        start=a;goal=b;
        intervals=2*static_cast<int>(std::ceil(sweep/step));
        found=true;
        break;
      }
      if (found) break;
    }
    const double elapsed=std::chrono::duration<double>(Clock::now()-began).count();
    if (!found)
    {
      report << "{\"passed\":false,\"stage\":\"candidate_search\",\"reason\":\"no_case_within_search_budget\",\"candidates_examined\":"
             << tried << ",\"search_wall_s\":" << elapsed << ",\"robot_motion_requested\":false}\n";
      RCLCPP_WARN(node->get_logger(),"No case found. Keep results; do not alter limits or move the robot.");
      rclcpp::shutdown();
      return 2;
    }
    std::ofstream csv(path/"direct_interpolation.csv");
    csv << "sample_index,alpha,bounds_valid,collision,box_collision,contact_pairs";
    for (const auto& n:expected) csv << ',' << n << "_rad";
    csv << '\n';
    int blocked=0;
    for (int i=0;i<=intervals;++i)
    {
      double alpha=static_cast<double>(i)/intervals;
      std::vector<double> q(6);
      for (std::size_t j=0;j<6;++j) q[j]=start[j]+alpha*(goal[j]-start[j]);
      auto result=check(q);
      if (result.hits_box) ++blocked;
      csv << i << ',' << std::setprecision(17) << alpha << ',' << result.bounded << ',' << result.collision << ',' << result.hits_box << ',' << std::quoted(result.contacts);
      for (double value:q) csv << ',' << std::setprecision(17) << value;
      csv << '\n';
    }
    std::ofstream candidate(path/"candidate.json");
    candidate << "{\n\"case_version\":\"detour_candidate_v1\",\n\"group\":" << std::quoted(group_name)
              << ",\n\"model_frame\":" << std::quoted(model->getModelFrame()) << ",\n\"obstacle_id\":" << std::quoted(box)
              << ",\n\"seed\":" << seed << ",\n\"joint_names\":[";
    for (std::size_t i=0;i<expected.size();++i) { if(i) candidate<<',';candidate<<std::quoted(expected[i]); }
    candidate << "],\n\"start_rad\":";vector_json(candidate,start);
    candidate << ",\n\"goal_rad\":";vector_json(candidate,goal);
    candidate << ",\n\"actual_robot_reset_to_start\":false,\n\"planned_detour_exists\":\"not_tested\"\n}\n";
    report << "{\"passed\":" << (blocked>0?"true":"false")
           << ",\"stage\":\"valid_endpoints_and_blocked_direct_interpolation\",\"start_valid\":true,\"goal_valid\":true,\"direct_samples\":"
           << intervals+1 << ",\"box_collision_samples\":" << blocked << ",\"max_per_joint_step_rad\":" << step
           << ",\"candidates_examined\":" << tried+1 << ",\"search_wall_s\":" << elapsed
           << ",\"robot_motion_requested\":false,\"scene_modified\":false,\"detour_planning_tested\":false}\n";
    RCLCPP_INFO(node->get_logger(),"Found valid endpoints and %d direct-interpolation box collision samples. SAVED: %s",blocked,output.c_str());
    rclcpp::shutdown();
    return blocked>0?0:2;
  }
  catch (const std::exception& error)
  {
    report << "{\"passed\":false,\"error\":" << std::quoted(error.what()) << ",\"robot_motion_requested\":false}\n";
    RCLCPP_ERROR(node->get_logger(),"%s",error.what());
    rclcpp::shutdown();
    return 2;
  }
}
