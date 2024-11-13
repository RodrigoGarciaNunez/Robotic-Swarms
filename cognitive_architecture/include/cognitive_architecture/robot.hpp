#include "robot.h"

Robot::Robot(int id , char tipo, int task) {
    id_ = id;
    tipo_ = tipo;
    task_ = task;
    dropOut = 0.0;
    p = new RobotNaviFun(id_, task_);
    
    corteza_motora_secundaria = make_shared<CmSec>(id_, tipo_, task_, dropOut);
    corteza_premotora = make_shared<cp>(id_,p,params,&bandera,task_);

    //cerr << corteza_premotora.get()->get_node_base_interface()->get_name() << endl;
}

Robot::~Robot() {}

void Robot::ejecutar() {
    rclcpp::spin(corteza_motora_secundaria);
    
    //rclcpp::spin(corteza_motora_secundaria);
}

void Robot::SleepLearning() {
    
    server = make_shared<srvEvaluateDriver>(1, 0, 0);
    
    thread serverThread([this] {
        rclcpp::spin(server);
    });

    thread cpThread([this] {
        corteza_premotora->ejecutaGenetico();
    });
    
    cpThread.join();
    serverThread.join();
}

void Robot::mirroring(int i) { // i representa al indivduo que se va a imitar
    
    thread cpThread([this, &i] {
        corteza_premotora->Mirroring(i);
    });

    cpThread.join();
    cerr << "Terminó el mirroring" << endl;
}

