#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/bool.hpp"
// #include "arlo_interfaces/msg/estado_arlo.hpp"

class SistemaLimbico : public rclcpp::Node
{
public:
    SistemaLimbico(int i, char tipo)
        : Node("sistema_limbico_" + std::to_string(i) + tipo), 
          identificador(i)
    {
        // Publicador para enviar impulsos motores a la área motora presuplementaria
        publisher_impulso_motor_ = this->create_publisher<std_msgs::msg::String>(
            "robot" + std::to_string(i) + tipo + "/impulso_motor", 10);

        // Timer para enviar impulsos motores periódicamente
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2), 
            std::bind(&SistemaLimbico::enviarImpulsoMotor, this));
    }

private:
    void enviarImpulsoMotor()
    {
        // Crear y enviar el impulso motor
        std_msgs::msg::String impulso_msg;
        impulso_msg.data = "Motivación para iniciar movimiento";
        
        publisher_impulso_motor_->publish(impulso_msg);
        RCLCPP_INFO(this->get_logger(), "Impulso motor enviado: %s", impulso_msg.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_impulso_motor_;
    rclcpp::TimerBase::SharedPtr timer_;
    int identificador;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nodo = std::make_shared<SistemaLimbico>(1, 'A'); 
    rclcpp::spin(nodo);
    rclcpp::shutdown();
    return 0;
}
