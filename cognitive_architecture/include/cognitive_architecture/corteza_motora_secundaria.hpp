#include "corteza_motora_secundaria.h"

// este nodo contiene la red neuronal que se encarga de decidir los movimientos del robot
// cmSec es corteza motora Secundaria

CmSec::CmSec(int i, char tipo, int task) : Node("Corteza_motora_secundaria_" + to_string(i) + tipo), identificador(i), task(task),
                                    redNeuronal(task_map[task][0], task_map[task][1], 0.0, rangos_salidas), tipo(tipo - 48) // -48 por el ascci
{
   flag_success = false;
   service_getWeights =  this->create_service<arlo_interfaces::srv::GetImportantWeights>("service_importantWeights", 
                bind(&CmSec::service_importantWeights,this, placeholders::_1, placeholders::_2));

   publisher_NN = this->create_publisher<std_msgs::msg::Float64MultiArray>
               ("robot" + to_string(i) + tipo + "/corteza_motora_secundariaNN", 10);

   publisher_evo_ = this->create_publisher<arlo_interfaces::msg::PesosStruct>
               ("robot" + to_string(i) + tipo + "/corteza_motora_secundaria_pesos", 10);

   subscriber_evo = this->create_subscription<std_msgs::msg::String>
               ("robot" + to_string(i) + tipo + "/corteza_premotora_evolutivo", 10, bind(&CmSec::setParameters_evo, this, placeholders::_1));

   publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>
               ("robot" + to_string(i) + tipo + "/corteza_motora_secundaria", 10);

   subscription_ = this->create_subscription<arlo_interfaces::msg::EstadoArlo>
               ("robot" + to_string(i) + tipo + "/temporal_lobe_", 1, bind(&CmSec::ejecutaNN, this, placeholders::_1));


   char archivo[50];

   if (string(1, tipo) == "1") sprintf(archivo, "./archivo_pesos_predeterminado_%d.txt", task);
   else sprintf(archivo, "./archivo_pesos_%d_%d.txt", identificador, task);

   if (!fileExists(archivo))
   {
      cerr << "no existe un archivo entrenado" << endl;
      genera_pesos(archivo);
   }

   redNeuronal.setParameters(archivo);
}
 
CmSec::~CmSec(){}

void CmSec::ejecutaNN(const arlo_interfaces::msg::EstadoArlo &msg)
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
         flag_success = true;
         // std::cerr << "si llegué" << std::endl;
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

void CmSec::setParameters_evo(const std_msgs::msg::String &msg) 
{
   // RCLCPP_INFO(this->get_logger(), "me llegaron mis nuevos pesos ->%s", msg.data.c_str());
   redNeuronal.setParameters(msg.data.c_str());
   flag_success = false;
}

bool CmSec::fileExists(string path)
{
   struct stat info;
   return (stat(path.c_str(), &info) == 0 && !(info.st_mode & S_IFDIR)); // comprueba la existencia de un archivo, no de un directorio
}

void CmSec::genera_pesos(const char *archivo_name)
{

   ofstream archivo(archivo_name);
   random_device rd;
   mt19937 gen(rd());                               // Motor mersenne_twister_engine
   uniform_int_distribution<> distribucion(1, 500); // Números entre 1 y 500

   archivo << to_string(task_map[task][0]) + " " + to_string(task_map[task][1]) + " 0\n";
   for (int i = 0; i < task_map[task][0]; i++)
   {
      archivo << distribucion(gen) << " " << distribucion(gen) << "\n";
   }
}

void CmSec::service_importantWeights(const std::shared_ptr<arlo_interfaces::srv::GetImportantWeights::Request> request,
                                     std::shared_ptr<arlo_interfaces::srv::GetImportantWeights::Response> response)
{

}
