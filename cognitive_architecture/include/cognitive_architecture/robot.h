#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include "cognitive_architecture/modulos_cerebrales/corteza_premotora.hpp"
#include "cognitive_architecture/modulos_cerebrales/corteza_motora_secundaria.hpp"
#include "cognitive_architecture/control_simulacion/srvEvaluateDriver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <thread>

using namespace std;

class Robot {

public:
    Robot(int i, char tipo, int task);
    ~Robot();

    void ejecutar();
    void SleepLearning();

    void mirroring(int i);

private:

    //rclcpp::Node::SharedPtr corteza_premotora;
    std::shared_ptr<cp> corteza_premotora;
    rclcpp::Node::SharedPtr corteza_motora_secundaria;
    rclcpp::Node::SharedPtr server;

    int id_;
    char tipo_;
    int task_;
    RobotNaviFun* p;
    ParamsGA params;
    bool bandera=true;
    double dropOut; 
};

#endif // ROBOT_H