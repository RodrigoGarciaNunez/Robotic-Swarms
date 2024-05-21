#ifndef ROBOTNAVIFUN_H
#define ROBOTNAVIFUN_H

#include "ProblemaOptim.h"

/*
Definición de la clase que modela la evaluación de un vector de pesos para
el controlador del robot.
La clase de debe derivar de la clase base ProblemaOptim.
*/

class RobotNaviFun : public ProblemaOptim {
public:
   RobotNaviFun(int maxSimTime = 50, double touchThreshold = 1);
   virtual ~RobotNaviFun();

   void evaluateFun(vector<double> const &x, double &fun, vector<double> &cons) const;

private:
   int numInputsNN;  // Number of inputs of the neural network.
   int numOutputsNN; // Number of outputs of the neural network.
   int numHidden;    // Number of hidden nodes of the neural network.
   int maxSimTime;
   double touchThreshold;
   string popFilePattern;  // Archivo de pesos para comunicación con el simulador.

   void writeWeightsFile(vector<double> const &weights, int id=1) const;
};


#endif
