#include "robot.h"

Robot::Robot(int id , char tipo, int task) {
    id_ = id;
    tipo_ = tipo;
    task_ = task;

    corteza_motora_secundaria = make_shared<CmSec>(id_, tipo_, task_);

    //corteza_premotora = make_shared<cp>(id_, p, params, &bandera, task_);
    //server = make_shared<srvEvaluateDriver>(task, 0, 0);
}

Robot::~Robot() {}

void Robot::ejecutar() {
    rclcpp::spin(corteza_motora_secundaria);
}

void Robot::SimulationSerever() {
    p = new RobotNaviFun(id_, task_);
    
    corteza_premotora = make_shared<cp>(id_,p,params,&bandera,task_);
    server = make_shared<srvEvaluateDriver>(1, 0, 0);
    
    thread serverThread([this] {
        rclcpp::spin(server);
    });

    thread cpThread([this] {
        //corteza_premotora.ejecutaGenetico();
        rclcpp::spin(corteza_premotora);
    });
    
    cpThread.join();
    serverThread.join();
}
