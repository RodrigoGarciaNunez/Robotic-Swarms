#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <random>
#include "cognitive_architecture/Algoritmo_Evolutivo/GeneticoSimple.hpp"
#include "arlo_interfaces/msg/pesos_struct.hpp"
#include "std_srvs/srv/empty.hpp"
//#include "ProblemaOptim.h"
#include "cognitive_architecture/Algoritmo_Evolutivo/RobotNaviFun.hpp"
//#include "Individuo.h"
//#include "Estadisticas.h"

using std::placeholders::_1;
// #include "cognitive_architecture/SimulationController.h"

// este nodo contiene el algoritmo evolutivo para los pesos de la red neuronal de la cms

class cp : public rclcpp::Node
{

public:

    cp(int i, RobotNaviFun* p, ParamsGA params,bool * bandera) : Node("Corteza_premotora_" + std::to_string(i)+"0"), 
    identificador(i), genetico(p, params), flag(bandera) 
    {

        this->genera_pesos();

        reset_simulation_client_ = this->create_client<std_srvs::srv::Empty>("/reset_world");

        //service_ = this->create_service<std_srvs::srv::Empty>("robot" + std::to_string(i) + "0_activacion", std::bind(&cp::ejecutaGenetico,this));

        publisher_ = this->create_publisher<std_msgs::msg::String>("robot" + std::to_string(i) + "0/corteza_premotora_evolutivo", 10);
        
        //subscription_ =
        //    this->create_subscription<arlo_interfaces::msg::PesosStruct>("robot" + std::to_string(i) + "0/corteza_motora_secundaria_pesos", 10, std::bind(&cp::ejecutaGenetico, this, std::placeholders::_1));

        
        timer_ = this -> create_wall_timer(700ms, std::bind(&cp::ejecutaGenetico,this));

        //this->ejecutaGenetico();
    }

private:

    // void on_activate() override {
    //     // Esta función se llama automáticamente cuando el nodo es activado
    //     RCLCPP_INFO(this->get_logger(), "Nodo activado");
    //     // Llama a la función deseada aquí
    //     ejecutaGenetico();
    // }

    void resetGazebo()const{
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto response = reset_simulation_client_->async_send_request(request);
    }

    //void callback(const arlo_interfaces::msg::PesosStruct &msg)
    void ejecutaGenetico()
    {
        RCLCPP_INFO(this->get_logger(), "me llegaron los pesos ");

        /*for (auto peso : msg.pesos)
        {
            std::cerr << peso << std::endl;
        }*/
        auto pesos = std_msgs::msg::String();

        this->genera_pesos();

        genetico.optimizar();


        //aca hay que ponerle una publicacion para colocar al mejor controlador

        std::cerr<< "voy a mandar los mejores pesos que obtuvo optimizar" << std::endl;
        

        auto mensajePesos = std_msgs::msg::String();

        //mensajePesos.data = "archivo_pesos_%d.txt", identificador;
        char buffer[50];
        std::sprintf(buffer, "./archivo_pesos_%d.txt", identificador);
        mensajePesos.data = std::string(buffer);

        publisher_->publish(mensajePesos);
        //resetGazebo();
        *flag=false;
    }

    void genera_pesos()
    {

        std::ofstream archivo("archivo_pesos_" + std::to_string(identificador) + ".txt"); // por el momento, al crearse este nodo, se crea un archivo con pesos aleatorios para cada robot
        std::random_device rd;
        std::mt19937 gen(rd());                             // Motor mersenne_twister_engine
        std::uniform_int_distribution<> distribucion(1, 3); // Números entre 1 y 100

        archivo << "98 2 0\n";
        for (int i = 0; i < 98; i++)
        {
            archivo << distribucion(gen) << " " << distribucion(gen) << "\n";
        }
    }



    // Atributos de la clase
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // Publicador para enviar los pesos óptimos
    rclcpp::Subscription<arlo_interfaces::msg::PesosStruct>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_; // Temporizador para ejecutar la optimización periódicamente
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_simulation_client_;
    rclcpp::Service<arlo_interfaces::srv::EvaluateDriver>::SharedPtr service_;
    bool* flag;
    GeneticoSimple genetico;
    //ProblemaOptim* p;
    int identificador;
};


/* Función principal
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Crear instancia de la clase cp con parámetros específicos
    auto node = std::make_shared<cp>(0, 50, 100, 0.8, 0.01); // Ejemplo de parámetros
    // Ejecutar el nodo
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}*/


// cp(int i, RobotNaviFun* p, ParamsGA params,bool * bandera) : Node("Corteza_premotora_" + std::to_string(i)+"0"), 
//     identificador(i), genetico(p, params), flag(bandera) 
//     {

//         this->genera_pesos();

//         reset_simulation_client_ = this->create_client<std_srvs::srv::Empty>("/reset_world");

//         publisher_ = this->create_publisher<std_msgs::msg::String>("robot" + std::to_string(i) + "0/corteza_premotora_evolutivo", 10);
//         //subscription_ =
//         //    this->create_subscription<arlo_interfaces::msg::PesosStruct>("robot" + std::to_string(i) + "0/corteza_motora_secundaria_pesos", 10, std::bind(&cp::callback, this, std::placeholders::_1));


//         //this->ejecutaGenetico();
//     }