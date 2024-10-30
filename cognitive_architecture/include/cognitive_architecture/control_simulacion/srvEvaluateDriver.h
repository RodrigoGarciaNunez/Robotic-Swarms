
#ifndef SRC_SIMULATIONCONTROLLER_H_
#define SRC_SIMULATIONCONTROLLER_H_


#include <rclcpp/rclcpp.hpp>
#include "SimulationState.h"
#include "arlo_interfaces/srv/evaluate_driver.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "arlo_interfaces/msg/estado_arlo.hpp"

using namespace std;

class srvEvaluateDriver: public rclcpp::Node {
public:

    srvEvaluateDriver(int task);
	virtual ~srvEvaluateDriver();
    SimulationState startSimulation(int maxtime);
    bool evaluateDriver(const std::shared_ptr<arlo_interfaces::srv::EvaluateDriver::Request> request,
                        const std::shared_ptr<arlo_interfaces::srv::EvaluateDriver::Response> response);

    void checkSimulationTime(const rosgraph_msgs::msg::Clock &msg);
    
    void checkModelPosition(const arlo_interfaces::msg::EstadoArlo &msg);
    

private:
    double dist2Go(double x, double y);
    void dist_to_mates(double x);
    double distance(double x1, double y1, double x2, double y2);
    void ejecutaSystem(const std::string& comando); 

    SimulationState arloState;
    double maxSimTime;  /* Maximum time allowed for the robot to get the goal */
    double goalDistance;
    double prev_x, prev_y;
    long double num_check; //numero de chequeos de odom;
    long int stuckCounter;
    bool stuck;
    int Task;

    rclcpp::Service<arlo_interfaces::srv::EvaluateDriver>::SharedPtr service_;
    rclcpp::Node::SharedPtr  nodoClock;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clockSus;
    rclcpp::Node::SharedPtr nodoOdom;
    rclcpp::Subscription<arlo_interfaces::msg::EstadoArlo>::SharedPtr OdomSus;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_pesos_eval;
    rclcpp::Node::SharedPtr clientg; 
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_simulation_client_;
    //rclcpp::
    //rclcpp::

    // auto clientg = rclcpp::Node::make_shared("cliente_reset");
    // auto reset_simulation_client_ = clientg->create_client<std_srvs::srv::Empty>("/reset_simulation");

    // auto requestg = std::make_shared<std_srvs::srv::Empty::Request>();
    // auto responseg = reset_simulation_client_->async_send_request(requestg);

    //rclcpp::executors::SingleThreadedExecutor::SharedPtr executor1;
    //rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_;
};

#endif