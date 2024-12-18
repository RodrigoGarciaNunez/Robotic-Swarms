#include "robot.h"

Robot::Robot(int id , char tipo, int task, int numMates, double goalx, double goaly) {
    id_ = id;
    tipo_ = tipo;
    task_ = task;
    numMates_= numMates;
    dropOut = 0.0;
    Goalx = goalx;
    Goaly = goaly;
    p = new RobotNaviFun(id_, task_);
    
    corteza_motora_secundaria = make_shared<CmSec>(id_, tipo_, task_, dropOut, Goalx, Goaly);
    corteza_premotora = make_shared<cp>(id_,p,params,&bandera,task_);

    MatesFitnessClient = std::make_shared<rclcpp::Node>("Get_mates_fitness_client_"+to_string(id_));
}

Robot::~Robot() {}

void Robot::ejecutar() {
    rclcpp::spin(corteza_motora_secundaria);    
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

void Robot::mirroring() { // i representa al indivduo que se va a imitar
    int id_toCopy = 0;
    double best_Fitness = 0;
    thread cpThread([this, &id_toCopy, &best_Fitness] {
        for(int i=1; i <= numMates_; i++){
            if(i != id_){
                service = MatesFitnessClient->create_client<arlo_interfaces::srv::GetMatesFitness>("robot"+to_string(i)+"0/service_get_fitness");
                auto request = std::make_shared<arlo_interfaces::srv::GetMatesFitness::Request>();
                auto result_future = service->async_send_request(request);
                if (rclcpp::spin_until_future_complete(MatesFitnessClient, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
                    auto result = result_future.get();
                    if(result->fitness > best_Fitness){
                        best_Fitness = result->fitness;
                        id_toCopy= result -> id;
                    }
                    

                } else {
                    cout << "Fallo al llamar al servicio para obtener pesos para imitación." << endl;
                }
                
            }
            
        }
        corteza_premotora->Mirroring(id_toCopy);
    });

    cpThread.join();
}

