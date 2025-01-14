#ifndef SRV_ED_MULTIPLE_GOALS
#define SRV_ED_MULTIPLE_GOALS


#include "srvEvaluateDriver.h"

class srvMultipleGoals: public srvEvaluateDriver
{
public:

    srvMultipleGoals(double x, double y, Miscelaneo * misc_);
    virtual ~srvMultipleGoals();
    SimulationState startSimulation(int maxtime);


private:

    const vector<pair<double, double>> GoalsCoordenates ={{0.0,0.0},{-5,5},{7,7},{9,0},{5,3},{3,7},{3,6}};
    int goalsUnreached=GoalsCoordenates.size();
    random_device rd;  // Semilla aleatoria
    mt19937 gen; // Generador Mersenne Twister
    uniform_int_distribution<> uniform_dist;
}

#endif