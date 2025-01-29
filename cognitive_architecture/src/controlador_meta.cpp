#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <fstream>
#include <cstdlib>
#include "cognitive_architecture/robot.hpp"

using namespace std;

int tipo;
int task;
int id;

// este controlador solo se encarga de activar el movimiento y al genético

int main(int argc, char **argv)
{

    setenv("GAZEBO_MASTER_URI", "http://localhost:11346", 1);  // 1 significa sobrescribir si ya existe
    setenv("ROS_DOMAIN_ID", "2", 1);

    if (argc < 4)
    {
        cerr << "Faltó un argumento: meta_exe tipo id task" << endl;

        return 0;
    }

    rclcpp::init(argc, argv);

    // std::signal(SIGUSR1, remSignal);
    tipo = atoi(argv[1]);
    id = atoi(argv[2]);
    task = atoi(argv[3]);

    Robot * robot = new Robot(id, *argv[1], task, 1, 0, 0);

    vector<thread> threads;

    threads.push_back(thread([&robot]()
                                 { robot->ejecutar(); }));
    

    cout << "Qué entrenamiento se va a realizar? 0 = Básico, 1 = Metas dinámicas, 2 = Básico c/ Dummies, 3 = Metas y Dummies Dinámicos";
    int elec;
    cin >> elec;
    robot->SleepLearning(elec);
        //robots[0].mirroring();

    for (auto &thread : threads)
        thread.join();
    rclcpp::shutdown();

    return 0;
}
