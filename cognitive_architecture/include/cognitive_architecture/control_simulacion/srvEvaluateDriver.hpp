#include "rclcpp/rclcpp.hpp"
#include "arlo_interfaces/srv/evaluate_driver.hpp"
#include "SimulationState.hpp"
// #include "srvEvaluateDriver.h"
#include "srvEvaluateDriver.h"
#include <unistd.h>
#include <thread>
//#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "rclcpp_components/node_factory.hpp"

srvEvaluateDriver::srvEvaluateDriver() : Node("servidor_simulacion")   //hay que agregar el id del robot que le corresponda
{
    rclcpp::QoS qos_profile(10);
    // Cambiar la política de confiabilidad a 'Best Effort'
    qos_profile.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    service_ = this->create_service<arlo_interfaces::srv::EvaluateDriver>("evaluate_driver", std::bind(&srvEvaluateDriver::evaluateDriver,
                                                                                                       this, std::placeholders::_1, std::placeholders::_2));

    nodoClock = std::make_shared<rclcpp::Node>("suscriptor_clock");
    clockSus = nodoClock->create_subscription<rosgraph_msgs::msg::Clock>("/clock", qos_profile, std::bind(&srvEvaluateDriver::checkSimulationTime, this, std::placeholders::_1));

    nodoOdom = std::make_shared<rclcpp::Node>("suscriptor_Odom");
    OdomSus = nodoOdom->create_subscription<nav_msgs::msg::Odometry>("robot10/odom", qos_profile, std::bind(&srvEvaluateDriver::checkModelPosition, this, std::placeholders::_1));

    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();


    publisher_pesos_eval = this->create_publisher<std_msgs::msg::String>("robot10/corteza_premotora_evolutivo", 10);

    RCLCPP_INFO(this->get_logger(), "Servidor listo para recibir solicitudes.");

    prev_x = 0;
    prev_y = 0;
    stuck = false;
    stuckCounter = 0;
}

srvEvaluateDriver::~srvEvaluateDriver()
{
}

bool srvEvaluateDriver::evaluateDriver(const std::shared_ptr<arlo_interfaces::srv::EvaluateDriver::Request> request,
                                       const std::shared_ptr<arlo_interfaces::srv::EvaluateDriver::Response> response)
{

    std::string pesos = request->weightsfile;

    std::cerr << pesos << std::endl;

    auto mensaje = std_msgs::msg::String();
    mensaje.data = pesos;

    publisher_pesos_eval->publish(mensaje);

    startSimulation(request->maxtime);

    response->time = arloState.finishTime;
    response->dist2go = arloState.distanceToGo;
    response->damage = arloState.robotDamage;
    response->energy = arloState.distanceTravelled;

    // for (auto& thread : threads) {
    //     thread.join();
    // }

    return true;
}

SimulationState srvEvaluateDriver::startSimulation(int maxtime)
{
    double fx=0;
    maxSimTime = maxtime;
    puts("Starting the simulation of a new driver...");
    puts("---------------------------");
    //std::cerr << maxtime << std::endl;
    //std::cerr << maxSimTime << std::endl;
    // std_srvs::Empty gazeboParams;


    auto clientg = rclcpp::Node::make_shared("cliente_reset");
    auto reset_simulation_client_ = clientg->create_client<std_srvs::srv::Empty>("/reset_simulation");

    auto requestg = std::make_shared<std_srvs::srv::Empty::Request>();
    auto responseg = reset_simulation_client_->async_send_request(requestg);
    sleep(1);


    arloState.resetState();
    stuckCounter = 0;
    int i = 0;
    while (rclcpp::ok() && !arloState.hasTimeRunOut && !arloState.finishLineCrossed)
    //while (rclcpp::ok() && i<20 && !arloState.finishLineCrossed)
    {
        executor->spin_node_once(nodoClock);
        executor->spin_node_once(nodoOdom);
        //i+=1;
        //sleep(1);
    }

    //if(i>=30){
    //    arloState.hasTimeRunOut=true;
    //}
    std::cout << "hasTimeRunOut= " << arloState.hasTimeRunOut << "\n";
    std::cout << "finishLineCrossed= " << arloState.finishLineCrossed << "\n";

    if (arloState.hasTimeRunOut == true)
    {
        arloState.finishTime = 2 * maxSimTime;
        // arloState.distanceToGo = goalDistance - arloState.currentPosition;
        cout << "currentPosition= " << arloState.currentPosition << "\n";
        if (arloState.stuck == true)
        {
            cout << " ---->>> ATASCADO  <<<-----" << endl;
            cout << " \t Counter = " << stuckCounter << endl;
        }

        fx = 25 + arloState.distanceToGo;
    }
    else
    { // The robot reached the goal.
        arloState.finishTime = arloState.currentTime;
        arloState.distanceToGo = 0.0;
        arloState.robotEnergy = 100;
        cout << "finishTime= " << arloState.finishTime << "\n";
        fx = arloState.finishTime;
    }

    cout << "x = " << arloState.position[0] << ", y = " << arloState.position[1] << endl;
    cout << "d2Go= " << arloState.distanceToGo << endl;
    cout << "gas= " << arloState.distanceTravelled << endl;
    cout << "fx= "<< fx <<endl;

    //response.time = arloState.finishTime;
    //response.dist2go = arloState.distanceToGo;
    //response.damage = arloState.robotDamage ;
    //response.energy = arloState.distanceTravelled;

    std::cout << "Bye." << std::endl;
    
    return arloState;
}

double srvEvaluateDriver::dist2Go(double x, double y)
{
    double distToGo = 0.0;
    //.  Si es la diana CEREBRAL
    double xTarget = 0.0;
    double yTarget = 0.0;
    distToGo = distance(xTarget, yTarget, x, y);

    return distToGo;
}

double srvEvaluateDriver::distance(double x1, double y1, double x2, double y2)
{
    double sum = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
    return sqrt(sum);
}

void srvEvaluateDriver::checkModelPosition(const nav_msgs::msg::Odometry &msg)
{
    // ROS_INFO("Seq: [%d]", msg->header.seq);
    // ROS_INFO("Position-> x: [%f]", msg->pose.pose.position.x);
    // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]",
    //		msg->twist.twist.linear.x,
    //		msg->twist.twist.angular.z);

    // arloState.currentPosition = msg->pose.pose.position.x;
    // if (arloState.currentPosition >= goalDistance)   // Esta es la distancia del pasillo en Gazebo.
    // 	arloState.finishLineCrossed = true;

    

    double distanceBefore = arloState.distanceTravelled;
    arloState.distanceTravelled += distance(prev_x, prev_y, msg.pose.pose.position.x, msg.pose.pose.position.y);

    if (abs(distanceBefore - arloState.distanceTravelled) < 0.01)
    {
        stuckCounter++;
        // cout << "Stuck counter: " << stuckCounter << "\n";
        if (stuckCounter > 80)
        {
            arloState.stuck = true;
            arloState.hasTimeRunOut = true;
        }
    }
    else
        stuckCounter = 0;

    prev_x = msg.pose.pose.position.x;
    prev_y = msg.pose.pose.position.y;

    arloState.currentPosition = msg.pose.pose.position.x;
    arloState.position[0] = msg.pose.pose.position.x;
    arloState.position[1] = msg.pose.pose.position.y;
    arloState.distanceToGo = dist2Go(msg.pose.pose.position.x, msg.pose.pose.position.y);
    if (arloState.distanceToGo <= 1.0) 
        arloState.finishLineCrossed = true;
}

void srvEvaluateDriver::checkSimulationTime(const rosgraph_msgs::msg::Clock &msg)
{
    arloState.currentTime = msg.clock.sec;

    //std::cerr << "################################tiempo##################################" << std::endl;
    //std::cout << "Tiempo simulacion: " << arloState.currentTime << std::endl;
    //std::cout << "maxSimTime: " << maxSimTime << std::endl;
    if (arloState.currentTime >= 25)
    {
        arloState.hasTimeRunOut = true;
        //std::cerr << "ya entre aqui  con " << arloState.currentTime << std::endl;
    }
}


void srvEvaluateDriver::ejecutaSystem(const std::string& comando){
    std::system(comando.c_str());
}

// int srvEvaluateDriver::getNumSensors() {
// 	return NUM_RAYS * NUM_SONARS;
// }

// int srvEvaluateDriver::getNumActuators() {
// 	return NUM_ACTUATORS;
// }
// rclcpp::Service<arlo_interfaces::srv::EvaluateDriver>::SharedPtr service_;
