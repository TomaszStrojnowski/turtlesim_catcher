#include <vector>
#include <cmath>//for hypot, atan2(prof. Fraczek to kocha), cos,sin
#include <limits>//to use the infinity 
#include <algorithm> //for range based loops
#include <chrono> //for timer

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
//my custom interfaces
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"



class TurtleControllerNode : public rclcpp::Node 
{
public:
    TurtleControllerNode() : Node("turtle_controller") 
    {
        //declaring parameters
        declare_parameter<double>("linear",2.0);
        declare_parameter<double>("angular",10.0);//with this angular velocity it works efficiently 
        declare_parameter<double>("catch_distance",1.0);
        declare_parameter<bool>("catch_closest",true);
        //getting parameters
        get_parameter("linear", linear_);
        get_parameter("angular", angular_);
        get_parameter("catch_distance",catch_distance_);
        get_parameter("catch_closest", catch_closest_);

        catch_turtle_ = create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");

        while (!catch_turtle_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Waiting for /catch_turtle service...");
        }

        pub_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);

        pose_sub_= create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,
                                       std::bind(&TurtleControllerNode::poseCallBack,this,std::placeholders::_1));
        alive_sub_=create_subscription<my_robot_interfaces::msg::TurtleArray>("alive_turtles",10,
                                        [this](my_robot_interfaces::msg::TurtleArray::SharedPtr msg){
                                            alive_turtles_ = msg->turtles;
                                        });

        timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                   std::bind(&TurtleControllerNode::control_loop,this));
    }
 
private:

    void poseCallBack(const turtlesim::msg::Pose::SharedPtr msg){
        current_pose_= *msg;
    }

    //void aliveTurtlesCallBack(const my_turtle_interfaces::msg::TurtleArray::SharedPtr msg){}

    void control_loop()
    {
        bool still_there = std::any_of(
            alive_turtles_.begin(), alive_turtles_.end(),
            [&](auto &t){ return t.name == target_turtle_name_; });
        if (!still_there) {
          target_turtle_name_.clear();
        }
        if(alive_turtles_.empty()) {
            return;
        }

        bool target_acquired = false;
        my_robot_interfaces::msg::Turtle selected;
        //loop that iterates through alive_turtles_ vector looking for turtle names that matches target_turtle_name_
        for(auto &t : alive_turtles_){ //auto &t binds t as a reference for each element in loop iteration
            if (t.name == target_turtle_name_){ //each t has a name that we compare with target_turtle_name
                target_acquired=true;//and now we choose a target 
                selected = t;
                break; //and we break out of the loop when we find our target
            }
        }
        //we only choose a new target at the beginning of the program or after weve caught one
        if(!target_acquired){
            if(catch_closest_){
                //then we create this best_d then loop over every turtle in alive_turtles_
                double best_d = std::numeric_limits<double>::infinity();
                for(auto &t : alive_turtles_){
                    double dx = t.x - current_pose_.x;
                    double dy = t.y - current_pose_.y;
                    //this hypot function computes the distance between two points in 2D in this case
                    //the distance between master turtle and its victim ;)
                    //for each turtle we compute the distance
                    double d = std::hypot(dx,dy);
                    if(d < best_d){
                        //and then we find the smallest distance and choose it as our target 
                        best_d = d;
                        selected = t;
                    }
                }

                
            }else{
                selected = alive_turtles_.front();
            }
            target_turtle_name_= selected.name;
        }

        double dx = selected.x - current_pose_.x; //how far the target is from master turtle in x axis
        double dy = selected.y - current_pose_.y; //how far the target is from master turtle in y axis
        double distance = std::hypot(dx,dy); //distance to target in staight line
        double angle_to_target = std::atan2(dy,dx); //we compute angle beetween the master turtle forward direction and the location of its target
        double heading_error = angle_to_target - current_pose_.theta;//this tells us how much the turtle needs to turtle to head the right direction

        heading_error = std::atan2(std::sin(heading_error),std::cos(heading_error));//we use this to normalize the eror into [-pi,pi]
        //P-controller(regulator proporcjonalny)
        geometry_msgs::msg::Twist cmd; 
        cmd.linear.x = linear_ * distance; //we set the forward velocity 
        cmd.angular.z = angular_ * heading_error;//we set rotational speed
        pub_->publish(cmd);//publishing this launch the killer turtle to its target

        if(distance < catch_distance_){
            auto req = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();//if we are close enough we create catch turtle service request
            req->name = target_turtle_name_;
            //we call it asynchronously to tell the spawner to kill thath turtle
            catch_turtle_->async_send_request(req,[](rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedFuture) {
            });
            target_turtle_name_.clear();//clearing the target name from the alive_turtles_
        }
    }

    
    
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_;

    rclcpp::TimerBase::SharedPtr timer_;

    turtlesim::msg::Pose current_pose_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_;
    std::string target_turtle_name_;
    //parameters
    bool catch_closest_;
    double linear_;
    double angular_;
    double catch_distance_;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}