#include <memory>
#include <string>
#include <vector>
#include <random>
#include <algorithm>
//built in libraries
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/msg/pose.hpp"
//my own custom interfaces
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

class TurtleSpawnerNode : public rclcpp::Node
{
public:
  TurtleSpawnerNode()
  : Node("turtle_spawner"), next_id_(1), gen_(rd_()), dist_(0.0, 11.0)
  {
    // Declare and get parameters
    declare_parameter<int>("max_turtles",5);
    declare_parameter<double>("spawn_frequency", 1.0);
    declare_parameter<std::string>("turtle_name_prefix", "turtle");
    get_parameter("max_turtles",max_turtles_);
    get_parameter("spawn_frequency", spawn_frequency_);
    get_parameter("turtle_name_prefix", turtle_name_prefix_);

    // Create clients
    spawn_client_ = create_client<turtlesim::srv::Spawn>("spawn");
    kill_client_  = create_client<turtlesim::srv::Kill>("kill");
    // QOL bullshit
    while (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Waiting for /spawn service...");
    }
    while (!kill_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Waiting for /kill service...");
    }

    pose_sub_ = create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,
                                                         std::bind(&TurtleSpawnerNode::masterPoseCallBack,this,std::placeholders::_1));

    // setting up a topic where alive turtles will be published as an array custom interface 
    alive_pub_ = create_publisher<my_robot_interfaces::msg::TurtleArray>("alive_turtles", 10);
    // Service for catching turtles
    catch_srv_ = create_service<my_robot_interfaces::srv::CatchTurtle>(
      "catch_turtle",
      std::bind(&TurtleSpawnerNode::catch_turtle, this, std::placeholders::_1, std::placeholders::_2)
    );
    // Timer spawning turtles at given frequency
    spawn_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / spawn_frequency_),
      std::bind(&TurtleSpawnerNode::spawn_turtle, this)
    );
  }

private:
  void spawn_turtle()
  {

    if((int)alive_turtles_.size()>=max_turtles_){
      return; //to stop too many turtles from spawning because the program didnt work efficeintly
    }

    double freq = spawn_frequency_ / (1 + alive_turtles_.size());
    auto period = std::chrono::duration<double>(1.0/freq);

    spawn_timer_->cancel();
    spawn_timer_=create_wall_timer(period, std::bind(&TurtleSpawnerNode::spawn_turtle,this));

    //generating random coordinates for spawning turtles 
    auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
    req->x     = dist_(gen_);
    req->y     = dist_(gen_);
    req->theta = 0.0;
    req->name  = turtle_name_prefix_ + std::to_string(next_id_++);

    //save request values
    double spawn_x        = req->x;
    double spawn_y        = req->y;
    std::string spawn_name = req->name;

    spawn_client_->async_send_request(
      req,
      [this, spawn_x, spawn_y](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
        auto res = future.get();  
        RCLCPP_INFO(
          get_logger(),
          "Spawned turtle '%s' at (%.2f, %.2f)",
          res->name.c_str(), spawn_x, spawn_y
        );
        // Build and store Turtle message
        my_robot_interfaces::msg::Turtle t;
        t.name = res->name;
        t.x    = spawn_x;
        t.y    = spawn_y;
        alive_turtles_.push_back(t);
        publish_alive_turtles();
      }
    );
  }

  void masterPoseCallBack(const turtlesim::msg::Pose::SharedPtr msg){
    master_pose_ = *msg;
  }

  void publish_alive_turtles()
  {
    {
      // Sort by distance to master_pose_
      std::sort(
        alive_turtles_.begin(),alive_turtles_.end(),[&](const auto &a, const auto &b) {
          double da = std::hypot(a.x - master_pose_.x, a.y - master_pose_.y);
          double db = std::hypot(b.x - master_pose_.x, b.y - master_pose_.y);
          return da < db;
        }
      );
    
      // Now we publish the sorted list
      my_robot_interfaces::msg::TurtleArray msg;
      msg.turtles = alive_turtles_;
      alive_pub_->publish(msg);
    }
  }  

  void catch_turtle(
    const std::shared_ptr<my_robot_interfaces::srv::CatchTurtle::Request> request,
    std::shared_ptr<my_robot_interfaces::srv::CatchTurtle::Response> response)
  {
    auto it = std::find_if( //searching for alive turtle in array
      alive_turtles_.begin(), alive_turtles_.end(),
      [&](const auto &t){ return t.name == request->name; }
    );
    if (it != alive_turtles_.end()) {
      auto kill_req = std::make_shared<turtlesim::srv::Kill::Request>();
      kill_req->name = it->name;
      kill_client_->async_send_request(kill_req);
      alive_turtles_.erase(it);
      publish_alive_turtles();
      response->success = true;
      RCLCPP_INFO(get_logger(), "Caught and killed turtle: '%s'", kill_req->name.c_str());
    } else {
      response->success = false;
      RCLCPP_WARN(get_logger(), "Turtle '%s' not found", request->name.c_str());
    }
  }

  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
  rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
  rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_pub_;
  rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_srv_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr spawn_timer_;
  turtlesim::msg:: Pose master_pose_;

  double spawn_frequency_;
  std::string turtle_name_prefix_;
  int next_id_;
  std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_;
  int max_turtles_;

  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> dist_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleSpawnerNode>());
  rclcpp::shutdown();
  return 0;
}
