#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <fstream>
// #include <csignal>
#include <cstdlib>
#include "cognitive_architecture/robot.hpp"
//#include "cognitive_architecture/miscelaneo.hpp"

using namespace std;

int tipo;
int task;
int num_bots;

int main(int argc, char **argv)
{

    if (argc < 4)
    {
        std::cerr << "Faltó un argumento: cpp_exe tipo num_bots task" << std::endl;

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
        if (senal == '1')
            robots[0].SleepLearning();
        else if (senal == '2')
        {
            robots[0].mirroring();
        }
    }

    for (auto &thread : threads)
    {
        thread.join();
    }
    rclcpp::shutdown();

    return 0;
}
