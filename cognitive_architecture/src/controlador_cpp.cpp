#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include "cognitive_architecture/robot.hpp"

using namespace std;

int tipo;
int task;
int num_bots;

int main(int argc, char **argv)
{

    setenv("GAZEBO_MASTER_URI", "http://localhost:11345", 1);  // 1 significa sobrescribir si ya existe
    setenv("ROS_DOMAIN_ID", "1", 1);
    // system("export GAZEBO_MASTER_URI=http://localhost:11345");
    // system("export ROS_DOMAIN_ID=1");

    if (argc < 4)
    {
        cerr << "Faltó un argumento: cpp_exe tipo num_bots task" << endl;

        return 0;
    }

    rclcpp::init(argc, argv);

    double Goalx, Goaly;
    cout << "Ingresa la coordenadas de la meta (x, y)";
    cin >> Goalx >> Goaly;
    cout << endl;

    // std::signal(SIGUSR1, remSignal);
    tipo = atoi(argv[1]);
    num_bots = atoi(argv[2]);
    task = atoi(argv[3]);

    vector<Robot> robots;

    // Crear nodos y agregarlos a la lista
    for (int i = 1; i <= num_bots; i++)
    {
        robots.push_back(Robot(i, *argv[1], task, num_bots, Goalx, Goaly));
    }

    vector<thread> threads;

    for (int i = 0; i < static_cast<int>(robots.size()); i++)
    {
        threads.push_back(thread([&robots, i]()
                                 { robots[i].ejecutar(); }));
    }

    if (tipo == 0)
    {
        char senal;
        cin >> senal;
        if (senal == '1'){
            threads.push_back (thread([]() 
                {   
                    system("export GAZEBO_MASTER_URI=http://localhost:11346");
                    system("export ROS_DOMAIN_ID=2");
                    system("ros2 launch cognitive_architecture meta_trainning.launch.py task:=1"); }));

            threads.push_back (thread([arg1 = string(argv[1]), arg3 = string(argv[3])]() 
                {   
                    system("export GAZEBO_MASTER_URI=http://localhost:11346");
                    system("export ROS_DOMAIN_ID=2");
                    ostringstream command;
                    command << "ros2 run cognitive_architecture meta_exe " << arg1 << " 1 " << arg3;
                    system(command.str().c_str()); }));

            // cout << "Qué entrenamiento se va a realizar? 0 = Básico, 1 = Metas dinámicas, 2 = Básico c/ Dummies, 3 = Metas y Dummies Dinámicos";
            // int elec;
            // cin >> elec;
            // robots[0].SleepLearning(elec);
        }
        else if (senal == '2') robots[0].mirroring();
    }

    for (auto &thread : threads) thread.join();
    rclcpp::shutdown();

    return 0;
}
