#include "srvED_Multiple_Goals.h"

srvMultipleGoals::srvMultipleGoals(double x, double y, Miscelaneo *misc_): Node("servidor_simulacion_Multiple_Goals")
{
    rclcpp::QoS qos_profile(1);
    // Cambiar la política de confiabilidad a 'Best Effort'
    qos_profile.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    service_ = this->create_service<arlo_interfaces::srv::EvaluateDriver>("evaluate_driver",
                                                                          bind(&srvEvaluateDriver::evaluateDriver, this, placeholders::_1, placeholders::_2));

    nodoClock = std::make_shared<rclcpp::Node>("suscriptor_clock");

    clockSus = nodoClock->create_subscription<rosgraph_msgs::msg::Clock>("/clock",
                                                                         qos_profile, bind(&srvEvaluateDriver::checkSimulationTime, this, placeholders::_1));

    nodoOdom = std::make_shared<rclcpp::Node>("suscriptor_Odom");

    OdomSus = nodoOdom->create_subscription<arlo_interfaces::msg::EstadoArlo>("robot10/temporal_lobe_",
                                                                              qos_profile, bind(&srvEvaluateDriver::checkModelPosition, this, placeholders::_1));

    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    publisher_pesos_eval = this->create_publisher<std_msgs::msg::String>("robot10/corteza_premotora_pesos", 10);

    ignoreSetter = this->create_publisher<std_msgs::msg::Int64>("robot10/ignoreFlag", 10);


    RCLCPP_INFO(this->get_logger(), "Servidor listo para recibir solicitudes.");

    clientg = rclcpp::Node::make_shared("cliente_reset");
    reset_simulation_client_ = clientg->create_client<std_srvs::srv::Empty>("/reset_simulation");

    prev_x = 0;
    prev_y = 0;
    stuck = false;
    stuckCounter = 0;
    Task = task;
    num_check = 0;
    x_start = -6.6;
    y_start = -13.6;

    gen = mt19937(random_device{}());
    gen.seed(rd());
    uniform_dist = uniform_int_distribution<>(0, goalsUnreached - 1);

    misc = misc_;
}

srvMultipleGoals::~srvMultipleGoals() {}

SimulationState srvMultipleGoals::startSimulation()
{

    puts("Starting the simulation of a new driver...");
    puts("---------------------------");

    arloState.resetState();
    double fx = 0;
    maxSimTime = maxtime;
    num_check = 0;

    stuckCounter = 0;

    while (goalsUnreached > 0)
    {
        current_goal_xy = GoalsCoordenates[uniform_dist(this->gen)];
        cout << current_goal_xy.first << "" << current_goal_xy.second << endl;

        misc->SetEntityState(current_goal_xy.first, current_goal_xy.second, 0.15, "cerebral_carpet",
                             "src/cognitive_architecture/models/cerebral_carpet/cerebral_carpet.sdf");

        while (rclcpp::ok() && !arloState.hasTimeRunOut && !arloState.finishLineCrossed)
        {
            executor->spin_node_once(nodoClock);
            executor->spin_node_once(nodoOdom);
        }

        cout << "hasTimeRunOut= " << arloState.hasTimeRunOut << "\n";
        cout << "finishLineCrossed= " << arloState.finishLineCrossed << "\n";
    }

    cout << "x = " << arloState.position[0] << ", y = " << arloState.position[1] << endl;
    cout << "d2Go = " << arloState.distanceToGo << endl;
    cout << "gas = " << arloState.distanceTravelled << endl;
    // cout << "dist_to_mates = " << arloState.dist_to_mates << endl;
    cout << "fx = " << fx << endl;

    cout << "Bye." << endl;

    return arloState;
}
