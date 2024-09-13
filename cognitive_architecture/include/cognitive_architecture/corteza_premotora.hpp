#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <random>
#include <sys/stat.h>

#include "cognitive_architecture/Algoritmo_Evolutivo/GeneticoSimple.hpp"
#include "arlo_interfaces/msg/pesos_struct.hpp"
#include "std_srvs/srv/empty.hpp"
// #include "ProblemaOptim.h"
#include "cognitive_architecture/Algoritmo_Evolutivo/RobotNaviFun.hpp"
// #include "Individuo.h"
// #include "Estadisticas.h"

using std::placeholders::_1;
// #include "cognitive_architecture/SimulationController.h"

// este nodo contiene el algoritmo evolutivo para los pesos de la red neuronal de la cms

class cp : public rclcpp::Node
{

public:
    cp(int i, RobotNaviFun *p, ParamsGA params, bool *bandera, int task) : Node("Corteza_premotora_" + std::to_string(i) + "0"),
                                                                 identificador(i), genetico(p, params, i, task), flag(bandera), task(task)
    {
        inputDir = "./archivo_pesos_"+std::to_string(identificador)+".txt";

        reset_simulation_client_ = this->create_client<std_srvs::srv::Empty>("/reset_world");

        publisher_ = this->create_publisher<std_msgs::msg::String>("robot" + std::to_string(i) + "0/corteza_premotora_evolutivo", 10);

        if(task==2){  //si la task a evaluar ocupa actualizar el numero de mates
            publisherMates = this->create_publisher<std_msgs::msg::Int64>("robot" + std::to_string(i) + "0/redefineMates", 10);
            auto actualizaMates = std_msgs::msg::Int64();
            actualizaMates.data = 2;  
            publisherMates->publish(actualizaMates);
        }
        
        // subscription_ =
        //     this->create_subscription<arlo_interfaces::msg::PesosStruct>("robot" + std::to_string(i) + "0/corteza_motora_secundaria_pesos", 10, std::bind(&cp::ejecutaGenetico, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(700ms, std::bind(&cp::ejecutaGenetico, this));

        // this->ejecutaGenetico();
    }

private:

    void resetGazebo() const
    {
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto response = reset_simulation_client_->async_send_request(request);
    }

    // void callback(const arlo_interfaces::msg::PesosStruct &msg)
    void ejecutaGenetico()
    {
        RCLCPP_INFO(this->get_logger(), "me llegaron los pesos ");

    
        /*for (auto peso : msg.pesos)
        {
            std::cerr << peso << std::endl;
        }*/
        auto pesos = std_msgs::msg::String();
        
        // if(!dirExists(inputDir)){
        //     std::cerr << "no hay archivo de pesos" << std::endl;
        //    this->genera_pesos();
        // }

        genetico.optimizar();

        // aca hay que ponerle una publicacion para colocar al mejor controlador

        std::cerr << "voy a mandar los mejores pesos que obtuvo optimizar" << std::endl;

        auto mensajePesos = std_msgs::msg::String();

        // mensajePesos.data = "archivo_pesos_%d.txt", identificador;
        char buffer[50];
        std::sprintf(buffer, "./archivo_pesos_%d_%d.txt", identificador,task);
        mensajePesos.data = std::string(buffer);

        publisher_->publish(mensajePesos);
        // resetGazebo();
        *flag = false;
    }


    // Atributos de la clase
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // Publicador para enviar los pesos óptimos
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisherMates;
    rclcpp::Subscription<arlo_interfaces::msg::PesosStruct>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_; // Temporizador para ejecutar la optimización periódicamente
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_simulation_client_;
    rclcpp::Service<arlo_interfaces::srv::EvaluateDriver>::SharedPtr service_;
    bool *flag;
    GeneticoSimple genetico;
    // ProblemaOptim* p;
    int identificador;
    int task;
    std::string inputDir;
};