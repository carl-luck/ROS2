#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iterator>
#include <cmath>
#include <cstring>
#include <stdexcept>

std::string read_text(const std::filesystem::path& path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("Cannot read "+path.string());
  return std::string(std::istreambuf_iterator<char>(stream),std::istreambuf_iterator<char>());
}

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  auto node=rclcpp::Node::make_shared("p3_check_spline_samples");
  auto input=node->declare_parameter<std::string>("input_dir","");
  auto path=std::filesystem::path(input);
  if(input.empty() || !std::filesystem::is_directory(path))
  {
    RCLCPP_ERROR(node->get_logger(),"Existing input_dir required");
    rclcpp::shutdown();return 2;
  }
  std::ofstream report(path/"spline_collision_report.json");
  try
  {
    auto urdf=read_text(path/"robot.urdf"),srdf=read_text(path/"robot.srdf");
    robot_model_loader::RobotModelLoader::Options options(urdf,srdf);
    options.load_kinematics_solvers_=false;
    robot_model_loader::RobotModelLoader loader(node,options);
    auto model=loader.getModel();
    if(!model) throw std::runtime_error("Model loading failed");
    const std::vector<std::string> names={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    auto group=model->getJointModelGroup("ur_manipulator");
    if(!group || group->getVariableNames()!=names) throw std::runtime_error("Unexpected joint order");
    std::ifstream file(path/"planning_scene.cdr",std::ios::binary);
    if(!file) throw std::runtime_error("Cannot read scene snapshot");
    std::vector<char> bytes((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());
    if(bytes.empty()) throw std::runtime_error("Empty scene snapshot");
    rclcpp::SerializedMessage serialized(bytes.size());
    auto& buffer=serialized.get_rcl_serialized_message();
    std::memcpy(buffer.buffer,bytes.data(),bytes.size());
    buffer.buffer_length=bytes.size();
    moveit_msgs::msg::PlanningScene msg;
    rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serializer;
    serializer.deserialize_message(&serialized,&msg);
    planning_scene::PlanningScene scene(model);
    if(!scene.setPlanningSceneMsg(msg)) throw std::runtime_error("Scene reconstruction failed");
    moveit::core::RobotState state(scene.getCurrentState());
    collision_detection::CollisionRequest request;
    request.group_name="ur_manipulator";
    request.contacts=true;request.max_contacts=100;request.max_contacts_per_pair=2;
    std::ifstream samples(path/"spline_samples.csv");
    if(!samples) throw std::runtime_error("Missing spline sample file");
    std::string line;
    std::getline(samples,line);
    std::string expected="time_from_start_s";
    for(const auto& n:names) expected+=","+n;
    if(line!=expected) throw std::runtime_error("Unexpected sample header/order");
    std::ofstream checks(path/"spline_sample_checks.csv");
    checks<<"sample_index,time_from_start_s,bounds_valid,collision,contact_pairs\n";
    std::size_t count=0,invalid=0;
    while(std::getline(samples,line))
    {
      std::stringstream stream(line);
      std::string field;
      std::vector<double> values;
      while(std::getline(stream,field,',')) values.push_back(std::stod(field));
      if(values.size()!=7) throw std::runtime_error("Malformed sample row");
      for(double value:values) if(!std::isfinite(value)) throw std::runtime_error("Nonfinite sample");
      std::vector<double> q(values.begin()+1,values.end());
      state.setJointGroupPositions(group,q);state.update();
      bool bounded=state.satisfiesBounds(group);
      collision_detection::CollisionResult result;
      scene.checkCollision(request,result,state);
      std::string pairs;
      for(const auto& contact:result.contacts)
      {
        if(!pairs.empty()) pairs+=';';
        pairs+=contact.first.first+"|"+contact.first.second;
      }
      checks<<count<<','<<std::setprecision(17)<<values[0]<<','<<bounded<<','<<result.collision<<','<<std::quoted(pairs)<<'\n';
      if(!bounded || result.collision) ++invalid;
      ++count;
    }
    bool passed=count>=2 && invalid==0;
    report<<"{\"passed\":"<<(passed?"true":"false")<<",\"sample_count\":"<<count<<",\"invalid_sample_count\":"<<invalid<<",\"robot_motion_requested\":false}\n";
    RCLCPP_INFO(node->get_logger(),"Spline samples: %zu, invalid: %zu. No movement.",count,invalid);
    rclcpp::shutdown();return passed?0:2;
  }
  catch(const std::exception& e)
  {
    report<<"{\"passed\":false,\"error\":"<<std::quoted(e.what())<<",\"robot_motion_requested\":false}\n";
    RCLCPP_ERROR(node->get_logger(),"%s",e.what());
    rclcpp::shutdown();return 2;
  }
}
