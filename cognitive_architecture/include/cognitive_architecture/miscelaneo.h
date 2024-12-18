#ifndef MISCELANEO_H
#define MISCELANEO_H

#include <iostream>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
using namespace std;

class Miscelaneo {

public:
    Miscelaneo();
    ~Miscelaneo();

    void SpawnEntity(string name, string NameSpace, string filem, double x, double y, double z);
    void SetEntityState(double x, double y, double z, string EntityName, string file_path);
    

private:
    rclcpp::Node::SharedPtr EntityManagerNode;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr EntitySpawnClient;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr EntitySetClient;

};

#endif 