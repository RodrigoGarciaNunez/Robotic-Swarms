#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "arlo_interfaces/msg/estado_arlo.hpp"

class AreaMotoraSupProp: public rclcpp::Node
{
public:
    AreaMotoraSupProp(int i, char tipo)
        : Node("area_motora_suplementaria_propia_" + std::to_string(i) + tipo), 
          identificador(i)
    {
        // Publicador para enviar el impulso motor a la área motora pre
        publisher_impulso_motor_ = this->create_publisher<std_msgs::msg::String>(
            "robot" + std::to_string(i) + tipo + "/impulso_motor", 10);

        // publisher_impulso = this->create_publisher<std_msgs::msg::String>("robot");

        // Timer para enviar el impulso motor periódicamente
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2), 
            std::bind(&AreaMotoraSupProp::enviarImpulsoMotor, this));
    }

private:
    void enviarImpulsoMotor()
    {
        // Crear y enviar el impulso motor
        std_msgs::msg::String impulso_msg;
        impulso_msg.data = "Impulso motor generado por el área motora suplementaria propia";

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
    auto nodo = std::make_shared<AreaMotoraSupProp>(1, 'A');
    rclcpp::spin(nodo);
    rclcpp::shutdown();
    return 0;
}
