#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <random>
#include "cognitive_architecture/Red_Neuronal/NeuroControllerDriver.hpp"
#include <cstdlib> // Necesario para system()
#include <cstdio>
//#include "arlo_interfaces/msg/pesos_struct.hpp"
#include "arlo_interfaces/msg/estado_arlo.hpp"
#include <cstring>
#include <sys/stat.h>
#include <map>

using std::placeholders::_1;
// using namespace std;

// este nodo contiene la red neuronal que se encarga de decidir los movimientos del robot
// cmSec es corteza motora Secundaria

static std::map<int, std::vector<int>> task_map = {
    {1, {98, 2}}, // 1.- Desplazamiento individula
    {2, {99, 2}}  // 2.- Desplazamiento en grupo
};

class cmSec : public rclcpp::Node
{

public:
   cmSec(int i, char tipo, int task) : Node("Corteza_motora_secundaria_" + std::to_string(i) + tipo), identificador(i), task(task),
      redNeuronal(task_map[task][0], task_map[task][1], rangos_salidas), tipo(tipo-48) // -48 por el ascci
   {

      //std::cerr << tipo <<std::endl;

      flag_success = false;

      publisher_NN = this->create_publisher<std_msgs::msg::Float64MultiArray>("robot" + std::to_string(i) + tipo + "/corteza_motora_secundariaNN", 10);

      publisher_evo_ = this->create_publisher<arlo_interfaces::msg::PesosStruct>("robot" + std::to_string(i) + tipo + "/corteza_motora_secundaria_pesos", 10);

      subscriber_evo = this->create_subscription<std_msgs::msg::String>("robot" + std::to_string(i) + tipo + "/corteza_premotora_evolutivo", 10, std::bind(&cmSec::callback_evo, this, std::placeholders::_1));

      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("robot" + std::to_string(i) + tipo + "/corteza_motora_secundaria", 10);

      subscription_ = this->create_subscription<arlo_interfaces::msg::EstadoArlo>("robot" + std::to_string(i) + tipo + "/temporal_lobe_", 10, std::bind(&cmSec::callback, this, std::placeholders::_1));

      char archivo[50];
      
      if (std::string(1, tipo) == "1") std::sprintf(archivo, "./archivo_pesos_predeterminado_%d.txt", task);
      else std::sprintf(archivo, "./archivo_pesos_%d_%d.txt", identificador, task);

      if (!fileExists(archivo))
      {
         std::cerr << "no existe un archivo entrenado" << std::endl;
         genera_pesos(archivo);
      }

      redNeuronal.setParameters(archivo);
   }

private:
   void callback(const arlo_interfaces::msg::EstadoArlo &msg)
   {
      if (flag_success == false)
      {
         std_msgs::msg::Float64MultiArray vector_reaction;

         for (size_t i = 0; i < msg.sensor1.ranges.size(); i++)
         {
            entradas.push_back(msg.sensor1.ranges[i]);
            entradas.push_back(msg.sensor2.ranges[i]);
            entradas.push_back(msg.sensor3.ranges[i]);
            entradas.push_back(msg.sensor4.ranges[i]);
         }


         dist_to_go_x = 0.0 - (msg.odom.pose.pose.position.x);
         dist_to_go_y = 0.0 - (msg.odom.pose.pose.position.y);

         entradas.push_back(dist_to_go_x);
         entradas.push_back(dist_to_go_y);

         if (task == 2)
         { // si la tarea involucra la distancia a los compañeros
            entradas.push_back(msg.dist_to_mates);
         }

         vector<double> reaction;

         redNeuronal.driveArlo(entradas, reaction);

         if (((dist_to_go_x <= 1) && (dist_to_go_x >= -1)) && ((dist_to_go_y <= 1) && (dist_to_go_y >= -1)) && (tipo == 0))
         {
            reaction.clear();
            reaction.push_back(0);
            reaction.push_back(0);
            //flag_success = true;
            //std::cerr << "si llegué" << std::endl;
         }

         vector_reaction.data = reaction;

         publisher_NN->publish(vector_reaction);

         for (auto mensaje : mensajes_recibidos)
         {
            publisher_->publish(mensaje.sensor1);
         }

         mensajes_recibidos.clear();
         entradas.clear();
      }
   }

   void callback_evo(const std_msgs::msg::String &msg) // este se debe encargar de setear los pesos a usar en la evaluacion
   {
      RCLCPP_INFO(this->get_logger(), "me llegaron mis nuevos pesos ->%s", msg.data.c_str());
      redNeuronal.setParameters(msg.data.c_str());
      flag_success = false;
   }

   bool fileExists(std::string path)
   {
      struct stat info;
      return (stat(path.c_str(), &info) == 0 && !(info.st_mode & S_IFDIR)); // comprueba la existencia de un archivo, no de un directorio
   }

   void genera_pesos(const char *archivo_name)
   {

      std::ofstream archivo(archivo_name);
      std::random_device rd;
      std::mt19937 gen(rd());                               // Motor mersenne_twister_engine
      std::uniform_int_distribution<> distribucion(1, 500); // Números entre 1 y 500

      archivo << std::to_string(task_map[task][0]) + " " + std::to_string(task_map[task][1]) + " 0\n";
      for (int i = 0; i < task_map[task][0]; i++)
      {
         archivo << distribucion(gen) << " " << distribucion(gen) << "\n";
      }
   }

   // float normalizar(float valor, float min, float max)
   // {  
   //    float normalized;
   //    if (valor < min) normalized = 0;
   //    else normalized = valor;
      
   //    return normalized;
   // }

   rclcpp::Subscription<arlo_interfaces::msg::EstadoArlo>::SharedPtr subscription_; // aqui se declara al suscriptor
   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_NN;
   rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
   rclcpp::Publisher<arlo_interfaces::msg::PesosStruct>::SharedPtr publisher_evo_;
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_evo;
   std::vector<arlo_interfaces::msg::EstadoArlo> mensajes_recibidos;
   int task;
   int id;
   int tipo;
   std::vector<double> entradas;
   int identificador;
   NeuroControllerDriver redNeuronal;
   vector<pair<double, double>> rangos_salidas;
   int banderaGenetico;
   double dist_to_go_x;
   double dist_to_go_y;
   bool flag_success;
};