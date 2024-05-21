
#ifndef SRC_SIMULATIONCONTROLLER_H_
#define SRC_SIMULATIONCONTROLLER_H_


#include <rclcpp/rclcpp.hpp>
#include "SimulationState.h"
#include "arlo_interfaces/srv/evaluate_driver.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

class srvEvaluateDriver: public rclcpp::Node {
public:

    srvEvaluateDriver();
	virtual ~srvEvaluateDriver();
    SimulationState startSimulation(int maxtime);
    bool evaluateDriver(const std::shared_ptr<arlo_interfaces::srv::EvaluateDriver::Request> request,
                        const std::shared_ptr<arlo_interfaces::srv::EvaluateDriver::Response> response);

    void checkSimulationTime(const rosgraph_msgs::msg::Clock &msg);
    
    void checkModelPosition(const nav_msgs::msg::Odometry &msg);
    

private:
    double dist2Go(double x, double y);
    double distance(double x1, double y1, double x2, double y2);
    void ejecutaSystem(const std::string& comando); 

    SimulationState arloState;
    double maxSimTime;  /* Maximum time allowed for the robot to get the goal */
    double goalDistance;
    double prev_x, prev_y;
    long int stuckCounter;
    bool stuck;

    rclcpp::Service<arlo_interfaces::srv::EvaluateDriver>::SharedPtr service_;
    rclcpp::Node::SharedPtr  nodoClock;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clockSus;
    rclcpp::Node::SharedPtr nodoOdom;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr OdomSus;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_pesos_eval;
    //rclcpp::executors::SingleThreadedExecutor::SharedPtr executor1;
    //rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_;
};

#endif