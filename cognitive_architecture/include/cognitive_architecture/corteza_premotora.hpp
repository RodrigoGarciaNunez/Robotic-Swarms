#include "corteza_premotora.h"

using std::placeholders::_1;
// #include "cognitive_architecture/SimulationController.h"

// este nodo contiene el algoritmo evolutivo para los pesos de la red neuronal de la cms

cp::cp(int i, RobotNaviFun *p, ParamsGA params, bool *bandera, int task) : Node("Corteza_premotora_" + std::to_string(i) + "0"),
                                                                           identificador(i), genetico(p, params, i, task), flag(bandera), task(task)
{
    inputDir = "./archivo_pesos_" + std::to_string(identificador) + ".txt";

    client_getWeights = this->create_client<arlo_interfaces::srv::GetImportantWeights>("service_importantWeights");

    reset_simulation_client_ = this->create_client<std_srvs::srv::Empty>("/reset_world");

    publisher_ = this->create_publisher<std_msgs::msg::String>("robot" + std::to_string(i) + "0/corteza_premotora_evolutivo", 10);

    if (task == 2)
    { // si la task a evaluar ocupa actualizar el numero de mates
        publisherMates = this->create_publisher<std_msgs::msg::Int64>("robot" + std::to_string(i) + "0/redefineMates", 10);
        auto actualizaMates = std_msgs::msg::Int64();
        actualizaMates.data = 1;
        publisherMates->publish(actualizaMates);
    }

    timer_ = this->create_wall_timer(700ms, std::bind(&cp::ejecutaGenetico, this));
}

cp::~cp() {}

void cp::resetGazebo() const
{
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto response = reset_simulation_client_->async_send_request(request);
}

void cp::ejecutaGenetico()
{
    RCLCPP_INFO(this->get_logger(), "me llegaron los pesos ");

    auto pesos = std_msgs::msg::String();

    genetico.optimizar();

    auto mensajePesos = std_msgs::msg::String();

    // mensajePesos.data = "archivo_pesos_%d.txt", identificador;
    char buffer[50];
    std::sprintf(buffer, "./archivo_pesos_%d_%d.txt", identificador, task);
    mensajePesos.data = std::string(buffer);

    publisher_->publish(mensajePesos);
    *flag = false;
}

void cp::Mirroing()
{
}
