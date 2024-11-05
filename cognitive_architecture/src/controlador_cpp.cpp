#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include "arlo_interfaces/msg/pesos_struct.hpp"
#include <csignal>
#include <cstdlib>
#include "cognitive_architecture/robot.hpp"
 

int tipo;
int task;
int num_bots;


int main(int argc, char **argv){
    

    if (argc < 4){
        std::cerr << "Faltó un argumento: cpp_exe tipo num_bots task" << std::endl;

        return 0;
    }

    rclcpp::init(argc, argv);

    std::signal(SIGUSR1, remSignal);
    tipo = std::atoi(argv[1]);
    num_bots = std::atoi(argv[2]);
    task = std::atoi(argv[3]);
    
    std::vector<Robot> robots;

    // Crear nodos y agregarlos a la lista
    for(int i=1; i< num_bots+1; i++){
        robots.push_back(Robot(i, *argv[1], task));
    }
    
    std::vector<std::thread> threads;
    
    for (int i=0; i< static_cast<int>(robots.size()); i++){
        threads.push_back(std::thread([&robots, i]() {
            robots[i].ejecutar();
        }));
    }

    
    if(tipo==0){
        char senal;
        std::cin >> senal;
        if (senal=='1') robots[0].SimulationSerever();

    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    rclcpp::shutdown();

    return 0;
}
