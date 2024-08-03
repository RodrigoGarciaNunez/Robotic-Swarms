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
#include "arlo_interfaces/msg/pesos_struct.hpp"
#include "arlo_interfaces/msg/estado_arlo.hpp"
#include <cstring>
#include <sys/stat.h>

using std::placeholders::_1;
// #include "cognitive_architecture/SimulationController.h"

// este nodo contiene la red neuronal que se encarga de decidir los movimientos del robot
// cmSec es corteza motora Secundaria

class cmSec : public rclcpp::Node
{

public:
   cmSec(int i, char tipo) : Node("Corteza_motora_secundaria_" + std::to_string(i) + tipo), identificador(i), redNeuronal(98, 2, rangos_salidas)
   {

      std::cerr << tipo << std::endl;
      publisher_NN = this->create_publisher<std_msgs::msg::Float64MultiArray>("robot" + std::to_string(i) + tipo + "/corteza_motora_secundariaNN", 10);
      publisher_evo_ = this->create_publisher<arlo_interfaces::msg::PesosStruct>("robot" + std::to_string(i) + tipo + "/corteza_motora_secundaria_pesos", 10);

      subscriber_evo =
          this->create_subscription<std_msgs::msg::String>("robot" + std::to_string(i) + tipo + "/corteza_premotora_evolutivo", 10, std::bind(&cmSec::callback_evo, this, std::placeholders::_1));

      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("robot" + std::to_string(i) + tipo + "/corteza_motora_secundaria", 10);

      subscription_ =
          this->create_subscription<arlo_interfaces::msg::EstadoArlo>("robot" + std::to_string(i) + tipo + "/temporal_lobe_", 10, std::bind(&cmSec::callback, this, std::placeholders::_1));
      // este de arriba debe recibir los valores de los 4 sensores y la posicion de arlo

      char archivo[50];
      if (std::string(1, tipo) == "1")
      {
         std::sprintf(archivo, "./archivo_pesos_predeterminado.txt");
      }
      else
      {
         std::sprintf(archivo, "./archivo_pesos_%d.txt", identificador);
      }

      if (!fileExists(archivo))
      {
         std::cerr << "no existe un archivo entrenado" << std::endl;
         genera_pesos();
      }

      redNeuronal.setParameters(archivo);

      // auto pesos = arlo_interfaces::msg::PesosStruct();
      // pesos.pesos = {1, 2, 3, 4, 5, 6, 5};
      // redNeuronal.vectorEnvia(pesos.pesos);
      // publisher_evo_->publish(pesos);
   }

private:
   void callback(const arlo_interfaces::msg::EstadoArlo &msg)
   {
      std_msgs::msg::Float64MultiArray vector_reaction;
      // RCLCPP_INFO(this->get_logger(), "me llego el mensaje %f", msg.range);
      // mensajes_recibidos.push_back(msg);

      // if (mensajes_recibidos.size() < 5)
      // { // espera a que le lleguen los 4 mensajes (4 sensores y 1 odom)
      //    return;
      // }

      for (int i = 0; i < msg.sensor1.ranges.size(); i++)
      {
         entradas.push_back(msg.sensor1.ranges[i]);
         entradas.push_back(msg.sensor2.ranges[i]);
         entradas.push_back(msg.sensor3.ranges[i]);
         entradas.push_back(msg.sensor4.ranges[i]);
      }

      entradas.push_back(0.0 - (msg.odom.pose.pose.position.x));
      entradas.push_back(0.0 - (msg.odom.pose.pose.position.y));

      // for (auto entrada : entradas)
      // {
      //    RCLCPP_INFO(this->get_logger(), "entrada %f", entrada);
      // }

      // aqui empieza la red
      // vector<pair<double, double>> dummy;
      vector<double> reaction;
      // NeuroControllerDriver redNeuronal(40, 2, dummy);
      // char archivo[50];
      // std::sprintf(archivo, "./archivo_pesos_%d.txt", identificador);
      // redNeuronal.setParameters(archivo);

      redNeuronal.driveArlo(entradas, reaction);

      //////////////////////// esto es para poder publicar el vector de la reaccion de la NN
      // set up dimensions
      vector_reaction.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      vector_reaction.layout.dim[0].size = reaction.size();
      vector_reaction.layout.dim[0].stride = 1;
      vector_reaction.layout.dim[0].label = "x";

      vector_reaction.data.clear();
      vector_reaction.data.insert(vector_reaction.data.end(), reaction.begin(), reaction.end());

      publisher_NN->publish(vector_reaction);
      ///////////////////////////////////////////////////////////////////////////////////////////////77

      for (auto mensaje : mensajes_recibidos)
      {
         publisher_->publish(mensaje.sensor1);
      }

      // auto pesos = std_msgs::msg::String();
      // pesos.data = "mande mis pesos";

      // auto pesos = arlo_interfaces::msg::PesosStruct();
      // pesos.pesos = {1, 2, 3, 4, 5, 6, 5};
      // redNeuronal.vectorEnvia(pesos.pesos);
      // publisher_evo_->publish(pesos);

      mensajes_recibidos.clear();
      entradas.clear();
   }

   void callback_evo(const std_msgs::msg::String &msg) // este se debe encargar de setear los pesos a usar en la evaluacion
   {
      RCLCPP_INFO(this->get_logger(), "me llegaron mis nuevos pesos ->%s", msg.data.c_str());
      redNeuronal.setParameters(msg.data.c_str());
   }

   bool fileExists(std::string path)
   {
      struct stat info;
      return (stat(path.c_str(), &info) == 0 && !(info.st_mode & S_IFDIR));  //comprueba la existencia de un archivo, no de un directorio
   }

   void genera_pesos()
   {

      std::ofstream archivo("archivo_pesos_" + std::to_string(identificador) + ".txt"); // por el momento, al crearse este nodo, se crea un archivo con pesos aleatorios para cada robot
      std::random_device rd;
      std::mt19937 gen(rd());                               // Motor mersenne_twister_engine
      std::uniform_int_distribution<> distribucion(1, 500); // Números entre 1 y 100

      archivo << "98 2 0\n";
      for (int i = 0; i < 98; i++)
      {
         archivo << distribucion(gen) << " " << distribucion(gen) << "\n";
      }
   }

   rclcpp::Subscription<arlo_interfaces::msg::EstadoArlo>::SharedPtr subscription_; // aqui se declara al suscriptor
   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_NN;
   rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
   rclcpp::Publisher<arlo_interfaces::msg::PesosStruct>::SharedPtr publisher_evo_;
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_evo;
   std::vector<arlo_interfaces::msg::EstadoArlo> mensajes_recibidos;
   std::vector<double> entradas;
   int identificador;
   NeuroControllerDriver redNeuronal;
   vector<pair<double, double>> rangos_salidas;
   int banderaGenetico;
};