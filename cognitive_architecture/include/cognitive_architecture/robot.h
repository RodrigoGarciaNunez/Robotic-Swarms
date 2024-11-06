#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include "cognitive_architecture/corteza_premotora.hpp"
#include "cognitive_architecture/corteza_motora_secundaria.hpp"
#include "cognitive_architecture/control_simulacion/srvEvaluateDriver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <thread>

using namespace std;

class Robot {
public:
    Robot(int i, char tipo, int task);
    ~Robot();

    void ejecutar();
    void SimulationSerever();

    private:
    rclcpp::Node::SharedPtr corteza_premotora;
    rclcpp::Node::SharedPtr corteza_motora_secundaria;
    rclcpp::Node::SharedPtr server;

    int id_;
    char tipo_;
    int task_;
    RobotNaviFun* p;
    ParamsGA params;
    bool bandera=true; 
};

#endif // ROBOT_H