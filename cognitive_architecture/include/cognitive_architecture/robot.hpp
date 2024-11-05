#include "robot.h"

Robot::Robot(int id , char tipo, int task) {
    p = new RobotNaviFun(id, task);
    corteza_premotora = make_shared<cp>(id,p,params,&bandera,task);
    corteza_motora_secundaria = make_shared<cmSec>(id, tipo, task);
    server = make_shared<srvEvaluateDriver>(task, 0, 0);
}

Robot::~Robot() {}

void Robot::ejecutar() {
    rclcpp::spin(corteza_motora_secundaria);
}

void Robot::SimulationSerever() {
    thread serverThread([this] {
        rclcpp::spin(server);
    });

    thread cpThread([this] {
        rclcpp::spin(corteza_premotora);
    });
    
    cpThread.join();
    serverThread.join();
}
