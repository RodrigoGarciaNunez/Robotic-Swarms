#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include "cognitive_architecture/corteza_premotora.hpp"
//#include "cognitive_architecture/Red_Neuronal/NeuroControllerDriver.hpp"
#include "cognitive_architecture/corteza_motora_secundaria.hpp"
#include "fmt/core.h"
#include "arlo_interfaces/msg/pesos_struct.hpp"
#include "cognitive_architecture/control_simulacion/srvEvaluateDriver.hpp"
#include <csignal>
#include <cstdlib>
#include "std_srvs/srv/empty.hpp"
 

int tipo;
int task;
int num_bots;

void remSignal(int signal){


    rclcpp::executors::SingleThreadedExecutor executor;

    bool bandera=true;    

    std::vector<std::shared_ptr<rclcpp::Node>> nodes_cp;
    
    for(int i=1; i<2; i++){
        RobotNaviFun* p = new RobotNaviFun(task);
        ParamsGA params;
        nodes_cp.push_back(std::make_shared<cp>(i, p, params, &bandera, task));
    }

    std::vector<std::thread> threadsc;

    //executor.spin();

    for(int i=0; i< static_cast<int>(nodes_cp.size()); i++){

        //service = client->create_client<std_srvs::srv::Empty>("robot%d0_activacion",i+1);
        
        threadsc.push_back(std::thread([&nodes_cp, i, &executor, &bandera]() {  //hay que checar que para más robots la bandera sea independiente
            while(bandera==true){
                //nodes_cp[i].ejecutar();
                executor.spin_node_once(nodes_cp[i]);
            }
            //rclcpp::spin(nodes_cp[i]);
            
        }));

    }   

    //executor

    std::shared_ptr<rclcpp::Node> server = std::make_shared<srvEvaluateDriver>(task);
    rclcpp::spin(server);

    for (auto& thread : threadsc) {
        thread.join();
    }

}


int main(int argc, char **argv){
    

    if (argc < 4){
        std::cerr << "Faltó un argumento: cpp_exe tipo num_bots task" << std::endl;

        return 0;
    }
    // Si en la línea de comandos se escribió un puerto, entonces asignarlo.
   
    //string URI = fmt::format("http://localhost:1135{}", 111);
    //setenv("ROS_MASTER_URI", URI.c_str(), 1);

    // Iniciar ROS en el puerto dado arriba.

    rclcpp::init(argc, argv);

    std::signal(SIGUSR1, remSignal);
    tipo = std::atoi(argv[1]);
    num_bots = std::atoi(argv[2]);
    task = std::atoi(argv[3]);
    

    std::vector<std::shared_ptr<rclcpp::Node>> nodes_cms;
    //std::vector<std::shared_ptr<rclcpp::Node>> nodes_cp;
    // Crear nodos y agregarlos a la lista
    for(int i=1; i< num_bots+1; i++){
        nodes_cms.push_back(std::make_shared<cmSec>(i,*argv[1],task));
    }
    
    std::vector<std::thread> threads;
    
    for (int i=0; i< static_cast<int>(nodes_cms.size()); i++){
        threads.push_back(std::thread([&nodes_cms, i]() {
            rclcpp::spin(nodes_cms[i]);
        }));
    }

    
    if(tipo==0){
        
        char senal;
        std::cin >> senal;

        if (senal=='1') std::raise(SIGUSR1);   

    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    rclcpp::shutdown();

    return 0;
}
