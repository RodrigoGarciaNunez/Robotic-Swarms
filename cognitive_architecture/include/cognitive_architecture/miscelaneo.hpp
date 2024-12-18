#include "miscelaneo.h"

Miscelaneo::Miscelaneo()
{
    EntityManagerNode = make_shared<rclcpp::Node>("Entity_Spawner");
    EntitySpawnClient = EntityManagerNode->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    EntitySetClient = EntityManagerNode->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
}

void Miscelaneo::SpawnEntity(string name, string NameSpace, string file_path, double x, double y, double z)
{   
    while (!EntitySpawnClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            std::cout << "Servicio Spawn Entity no disponible. Saliendo." << std::endl;
            return;
        }
        std::cout << "Servicio para Spawn Entity no disponible. Intentando nuevamente..." << std::endl;
    }

    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();

    ifstream file(file_path);
    if (!file.is_open())
    {
        RCLCPP_ERROR(EntityManagerNode->get_logger(), "No se pudo abrir el archivo: %s", file_path.c_str());
        return;
    }

    string model_xml((istreambuf_iterator<char>(file)), istreambuf_iterator<char>()); // esto lee mejor el archivo
    file.close();

    request->name = name; // Nombre único para el modelo
    request->xml = model_xml;
    request->robot_namespace = NameSpace;
    request->initial_pose.position.x = x;
    request->initial_pose.position.y = y;
    request->initial_pose.position.z = z;

    auto future = EntitySpawnClient->async_send_request(request);

    if (rclcpp::spin_until_future_complete(EntityManagerNode, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        RCLCPP_INFO(EntityManagerNode->get_logger(), "Modelo spawneado exitosamente: %s", request->name.c_str());
    }
    else
    {
        //auto response = future.get();
        RCLCPP_ERROR(EntityManagerNode->get_logger(), "Error al spawnear el modelo");
    }
}

void Miscelaneo::SetEntityState(double x, double y, double z, string EntityName, string file_path)
{
    while (!EntitySetClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            std::cout << "Servicio para Borrar entidad no disponible. Saliendo." << std::endl;
            return;
        }
        std::cout << "Servicio para Borrar entidad no disponible. Intentando nuevamente..." << std::endl;
    }

    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = EntityName;
    
    auto future = EntitySetClient->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(EntityManagerNode, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(EntityManagerNode->get_logger(), "Modelo borrado exitosamente: ");
    }
    else
    {
        RCLCPP_ERROR(EntityManagerNode->get_logger(), "Error al borrar el modelo:"); 
    }

    SpawnEntity(EntityName, EntityName, file_path, x, y , z);
    //////////////// lo comentado es la versión para actualizar la posición de la meta,
    //////////////// pero parece ser un problema que ros2 no ha podido resolver

    // request->model_state.model_name = EntityName;   //Nombre de la entidad a setear
    // request->model_state.pose.position.x=x;
    // request->model_state.pose.position.y=y;
    // request->model_state.pose.position.z=z;
    // request->model_state.reference_frame="world";
    
}