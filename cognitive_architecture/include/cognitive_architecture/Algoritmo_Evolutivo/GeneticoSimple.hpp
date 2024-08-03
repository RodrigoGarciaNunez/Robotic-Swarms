#include <rclcpp/rclcpp.hpp> // Incluye la biblioteca ROS 2
#include <ctime>
#include <cmath>
#include <limits>
#include <algorithm>
#include <sys/stat.h>
#include "GeneticoSimple.h" // Incluye la definición de clases y funciones relacionadas con el algoritmo genético

// Definición de la clase GeneticoSimple
// Constructor de la clase
GeneticoSimple::GeneticoSimple(ProblemaOptim *p, ParamsGA &params, int id)
{
    // Inicialización de parámetros y variables
    problema = p;
    popSize = params.popSize;
    Gmax = params.Gmax;
    Pc = params.Pc;
    Pm = params.Pm;
    precision = params.precision;

    random_device rd;
    rng = std::mt19937(std::random_device{}());
    rng.seed(rd());

    oldpop = new Individuo[popSize];
    newpop = new Individuo[popSize];
    padres.assign(popSize, 0);
    stats.reset(problema, precision);

    valores = {5, 2, 10, 1, 4};

    outputDir = "./salidafinal/";
    inputDir = "./archivo_pesos_" + std::to_string(id) + ".txt";
    // Comprobación y creación del directorio de salida
    if (!dirExists(outputDir))
    {
        const int dir_err = mkdir(outputDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (dir_err == -1)
        {
            cerr << "\nNo se pudo crear el directorio de salida " << outputDir << "\n\n";
            exit(1);
        }
    }
    std::cerr << "hice el constructor" << std::endl;
    // Evaluación de la población inicial y aplicación del elitismo
    // evaluarPoblacion(oldpop);
    // elitismo(oldpop, gen);

    // Optimización del algoritmo genético
    // optimizar();
}

// Destructor de la clase
GeneticoSimple::~GeneticoSimple()
{
    delete[] newpop;
    delete[] oldpop;
}

// Método para verificar si un directorio existe
bool GeneticoSimple::dirExists(std::string path)
{
    struct stat info;

    if (stat(path.c_str(), &info) != 0)
        return false;
    else if (info.st_mode & S_IFDIR)
        return true;
    else
        return false;
}

bool GeneticoSimple::fileExists(std::string path)
   {
      struct stat info;
      return (stat(path.c_str(), &info) == 0 && !(info.st_mode & S_IFDIR));  //comprueba la existencia de un archivo, no de un directorio
   }

// Método principal para la optimización utilizando el algoritmo genético
void GeneticoSimple::optimizar()
{
    Individuo *temp;
    // Reporte inicial de estadísticas
    // stats.initial_report(this->get_logger(), popSize, Gmax, Pc, Pm);
    std::cerr << "entre a optimizar" << std::endl;
    // Inicialización de la población
    gen = 1;
    inicalizarPob();
    evaluarPoblacion(oldpop);
    elitismo(oldpop, gen);
    stats.statistics(oldpop, popSize);

    std::cerr << "incialice la poblacion" << std::endl;
    // Bucle principal de optimización
    for (gen = 2; gen <= Gmax; gen++)
    {
        seleccionPadres(oldpop);
        // std::cerr << "seleccione padres" << std::endl;
        cruza(oldpop, newpop);
        // std::cerr << "hice cruza" << std::endl;
        mutacion(newpop);
        // std::cerr << "hice mutacion" << std::endl;
        evaluarPoblacion(newpop);
        // std::cerr << "hice evalue la poblacion" << std::endl;
        elitismo(newpop, gen);
        // std::cerr << "hice elitismo" << std::endl;
        stats.statistics(newpop, popSize);
        // std::cerr << "penultimo" << std::endl;
        stats.shortReport(cout, oldpop, newpop, popSize, gen);
        // std::cerr << "ultimo" << std::endl;
        //  Intercambio de poblaciones
        temp = oldpop;
        oldpop = newpop;
        newpop = temp;
    }

    // std::cerr << "sali el bucle" << std::endl;

    // Escritura de resultados en archivos de salida
    std::ofstream archVariables(outputDir + "/pesos_pob.txt", std::ofstream::out);
    std::ofstream archEvaluacion(outputDir + "/eval_pob.txt", std::ofstream::out);

    stats.writeVariables(archVariables, oldpop, popSize);
    stats.writeEvaluation(archEvaluacion, oldpop, popSize);

    archVariables.close();
    archEvaluacion.close();

    // std::terminate();
}

// Método para inicializar la población
void GeneticoSimple::inicalizarPob()
{

    if (fileExists(inputDir))
    {
        std::cerr << "hola, ya existe un controlador previamente entrenado llamado" << inputDir << std::endl;
        
        for(unsigned j=0 ; j < popSize; j++)
        {
            oldpop[j].iniciaInfo(problema, precision);
            newpop[j].iniciaInfo(problema, precision);
        }


        

        ifstream weightsFile;
        weightsFile.open(inputDir, std::ifstream::in);
        
        char buffer[7];
        weightsFile.getline(buffer, 7);

        double peso;
        unsigned k=0;
        while (weightsFile >> peso) {
            oldpop[0].x[k]=peso;
            newpop[0].x[k]=peso;
            k++;
        }

        weightsFile.close();

        std::cerr << "impresion de los pesos leidos" << std::endl;
        std::cerr << "tamaño de x " << oldpop[0].x.size() <<std::endl;

        for(int i=0; i<oldpop[0].x.size(); i++){
            std::cerr << oldpop[0].x[i] << " ";
        }
        std::cerr << std::endl;

        //std::cerr << "pase de inicia Info" << std::endl;
        oldpop[0].x2cromosoma();

        std::copy(oldpop[0].cromo.begin(), oldpop[0].cromo.end(), std::ostream_iterator<int>(std::cerr, " "));
        std::cerr << std::endl;
        //std::cerr << "pase de x2cromo" << std::endl;

        for (unsigned j=1; j < popSize; j++)
        {
            //std::cerr << j << std::endl;
            oldpop[j].copiar(&oldpop[0]);
            //std::cerr << j << std::endl;
            newpop[j].copiar(&oldpop[0]);
            //std::cerr << j << std::endl;
        }
        
        auxPm = Pm;
        Pm = 0.05;       /* mutación muy pequeña para dar un poco de variedad a la población */
        mutacion(oldpop);
        Pm = auxPm;       

    }

    else
    {
        for (int j = 1; j < popSize; j++)
        {
            //std::cerr << j << std::endl;
            oldpop[j].insuflar(problema, precision);
            //std::cerr << j << std::endl;
            newpop[j].insuflar(problema, precision);
            //std::cerr << j << std::endl;
        }
    }

    // Si Pm no tiene un valor asignado, se calcula como 1 dividido por el tamaño del cromosoma
    if (Pm == SIN_VALOR)
        Pm = 1.0 / oldpop[0].chromoSize;
}

// Método para evaluar la población
void GeneticoSimple::evaluarPoblacion(Individuo *pop)
{

    // std::system("ros2 launch cognitive_architecture dummys.launch.py  > /dev/null");

    // std::vector<std::string> comandos = {
    //     // dev/null es para que no se imprima en terminal
    //     "ros2 run cognitive_architecture cpp_exe 1 > /dev/null",
    //     "ros2 run cognitive_architecture Controlador_individuo.py 1 /dev/null"};

    // std::vector<std::thread> threads;

    // // Crear un hilo para cada comando en la lista
    // for (const auto &comando : comandos)
    // {
    //     threads.emplace_back([this, comando]()
    //                          { std::system(comando.c_str()); });
    // }

    // for (auto &thread : threads)
    // {
    //     thread.detach();
    // }

    for (int i = 0; i < popSize; ++i)
    {
        problema->evaluateFun(pop[i].x, pop[i].eval, pop[i].cons);
        stats.nevals++;

        pop[i].aptitud = 1.0 / (1 + pop[i].eval);
    }

    // std::terminate();
}

// Método para seleccionar los padres utilizando el método de la ruleta
void GeneticoSimple::seleccionPadres(Individuo *pop)
{
    calcularValEsperado(pop);

    for (int i = 0; i < popSize; i++)
        padres[i] = seleccionRuleta(pop);
}

// Método para seleccionar un padre utilizando el método de la ruleta
int GeneticoSimple::seleccionRuleta(Individuo *pop)
{
    double suma, ale;
    int j;
    std::uniform_real_distribution<> rdis(0.0, sumvalesp);

    ale = rdis(rng);
    suma = 0.0;
    j = 0;

    do
    {
        suma += pop[j].valesp;
        j++;
    } while (suma < ale && j < popSize);

    return j - 1;
}

// Método para realizar la cruza entre los individuos de la población
void GeneticoSimple::cruza(Individuo *oldpop, Individuo *newpop)
{
    int mate1, mate2, pcruza;

    for (int j = 0; j < popSize - 1; j += 2)
    {
        mate1 = padres[j];
        // std::cerr << "ciclo mate1 " << j << std::endl;
        mate2 = padres[j + 1];
        // std::cerr << "ciclo mate2 " << j+1 << std::endl;

        pcruza = cruza1Punto(oldpop[mate1].cromo, oldpop[mate2].cromo, newpop[j].cromo, newpop[j + 1].cromo);
        // std::cerr << "pase de cruza1punto" << j << std::endl;
        newpop[j].xsite1 = pcruza;
        newpop[j + 1].xsite1 = pcruza;
        newpop[j].padres[0] = mate1 + 1;
        newpop[j].padres[1] = mate2 + 1;
        newpop[j + 1].padres[0] = mate1 + 1;
        newpop[j + 1].padres[1] = mate2 + 1;

        // std::cerr << "ciclo cruza" << j << std::endl;
    }
}

// Método para realizar la cruza de un punto entre dos cromosomas
int GeneticoSimple::cruza1Punto(Cromosoma &padre1, Cromosoma &padre2, Cromosoma &hijo1, Cromosoma &hijo2)
{
    int pcruza, j;
    int chromoSize = padre1.size();
    std::uniform_int_distribution<int> unif(0, chromoSize - 1);
    // std::uniform_int_distribution<int>* unif = new std::uniform_int_distribution<int>(0, chromoSize-1);
    // std::cerr << "entre a cruza1punto"  << std::endl;

    if (flip(Pc))
    {
        // std::cerr << "entre al if" << std::endl;
        pcruza = unif(rng);
        // std::cerr << "uniform distribution" << std::endl;
        for (j = (chromoSize - 1); j >= (chromoSize - pcruza); j--)
        {
            // std::cerr << "ciclo for1 cruza1punto" << j << std::endl;
            hijo1[j] = padre1[j];
            hijo2[j] = padre2[j];
            // std::cerr << "ciclo for1 cruza1punto" << j << std::endl;
        }

        for (j = (chromoSize - pcruza) - 1; j >= 0; j--)
        {
            // std::cerr << "ciclo for2 cruza1punto" << j << std::endl;
            hijo1[j] = padre2[j];
            hijo2[j] = padre1[j];
        }

        stats.ncruzas++;
    }
    else
    {
        // std::cerr << "entre al else" << std::endl;
        hijo1 = padre1;
        // std::cerr << "voy bien 1" << std::endl;
        hijo2 = padre2;
        // std::cerr << "voy bien 2" << std::endl;
        pcruza = 0;
    }
    // delete unif;
    return pcruza;
}

// Método para realizar la mutación de la población
void GeneticoSimple::mutacion(Individuo *pop)
{
    for (int j = 0; j < popSize; j++)
    {
        pop[j].nMutaciones = mutacionUniforme(pop[j].cromo);
        std::cerr << "voy bien "<< std::endl;
        pop[j].decodificar();
        std::cerr << "mamé "<< std::endl;

        if (pop[j].nMutaciones > 0){
            stats.nmutaciones++;
        }
            
        std::cerr << "j " << j << std::endl;
    }

    std::uniform_int_distribution<int> unif(0, popSize - 1);
    int ale = unif(rng);
    pop[ale].copiar(&stats.bestfit);
    stats.positionBestFit = ale + 1;
}

// Método para realizar la mutación uniforme en un cromosoma
int GeneticoSimple::mutacionUniforme(Cromosoma &cromo)
{
    int numMutations = 0;
    std::cerr << "cromosize " << cromo.size() << std::endl;
    for (unsigned k = 0; k < cromo.size(); k++)
    {
        if (flip(Pm))
        {
            numMutations++;
            cromo[k] = (cromo[k] == 0) ? 1 : 0;
        }
        //std::cerr << "k " << k << std::endl;
    }

    return numMutations;
}

// Método para calcular el valor esperado de cada individuo de la población
void GeneticoSimple::calcularValEsperado(Individuo *pop)
{
    double sumaptitud = 0.0;

    for (int j = 0; j < popSize; j++)
        sumaptitud = sumaptitud + pop[j].aptitud;

    stats.avgApt = sumaptitud / popSize;

    sumvalesp = 0.0;

    for (int j = 0; j < popSize; j++)
    {
        if (stats.avgApt != 0.0)
        {
            pop[j].valesp = pop[j].aptitud / stats.avgApt;
        }
        else
        {
            pop[j].valesp = 0.0f;
        }
        sumvalesp += pop[j].valesp;
    }
}

// Método para aplicar elitismo en la población
void GeneticoSimple::elitismo(Individuo *pop, int gen)
{
    for (int j = 0; j < popSize; j++)
    {
        if (pop[j].aptitud > stats.bestfit.aptitud)
        {
            stats.bestfit.copiar(&pop[j]);
            stats.generationBestFit = gen;
            stats.positionBestFit = j + 1;
        }
    }
}

// Método para realizar un flip basado en una probabilidad
int GeneticoSimple::flip(double prob)
{
    std::uniform_real_distribution<> rdis(0.0, 1.0);

    if (rdis(rng) <= prob)
        return true; //
    else
        return false;
}

/*/ Función principal
int main(int argc, char* argv[]) {
    // Inicialización de ROS 2
    rclcpp::init(argc, argv);
    // Creación de un nodo GeneticoSimpleROS2
    auto node = std::make_shared<GeneticoSimpleROS2>("genetico_simple", nullptr, ParamsGA());
    // Ciclo de ejecución de ROS 2
    rclcpp::spin(node);
    // Cierre de ROS 2
    rclcpp::shutdown();
    return 0;
}*/

// /*
// * GeneticoSimple.cpp
// * Implemetación de un algoritmo genético simple con representación binaria,
// * selección proporcional de ruleta, cruza en un punto, y mutación uniforme.
// */

// #include <ctime>
// #include <cmath>
// #include <limits>
// #include <omp.h>
// #include "GeneticoSimple.h"

// #include <fmt/core.h>
// #include <fmt/ranges.h>
// #include <algorithm>
// #include <sys/stat.h>

// GeneticoSimple::GeneticoSimple(ProblemaOptim* p, ParamsGA& params) {
//    problema = p;
//    popSize = params.popSize;
//    Gmax = params.Gmax;
//    Pc = params.Pc;
//    Pm = params.Pm;
//    precision = params.precision;

//    random_device rd;
//    rng.seed(rd());
//    oldpop = new Individuo[popSize];
//    newpop = new Individuo[popSize];
//    padres.assign(popSize, 0);
//    stats.reset(problema, precision);

//    tiempoMaximo = 0.0;  // el PEOR tiempo en terminar la carrera.

//    valores = {5, 2, 10, 1, 4};

//    outputDir = "./salidafinal/";
//    if ( !dirExists(outputDir) ) {
//       const int dir_err = mkdir(outputDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
//       if (dir_err == -1 ) {
//          cerr << "\nNo se pudo crear el directorio de salida " << outputDir << "\n\n";
//          exit(1);
//       }
//    }
// }

// GeneticoSimple::~GeneticoSimple() {
//    delete[] newpop;
//    delete[] oldpop;
// }

// bool GeneticoSimple::dirExists(string path) {
//     struct stat info;

//     if(stat( path.c_str(), &info ) != 0)
//         return false;
//     else if(info.st_mode & S_IFDIR)
//         return true;
//     else
//         return false;
// }

// // bool GeneticoSimple::operator () (const int &a, const int &b) {
// //         return valores[a] > valores[b];
// // }

// void GeneticoSimple::migrar() {
//     vector<int> elegidos(5, 0);
//     for (int i = 0; i < elegidos.size(); i++)
//         elegidos[i] = i;

// //    sort(elegidos.begin(), elegidos.end(), Less(*this));
//     sort(elegidos.begin(), elegidos.end(), [this](auto i, auto j) {return valores[i] > valores[j];});

//     for (auto elem : elegidos)
//         cout << elem << ", ";

//     cout << endl;
// }

// void GeneticoSimple::optimizar()
// {
//    Individuo* temp;

//    /* Inicializar la población y reportar las estadísticas iniciales */
//    gen = 1; /* La generación aleatoria es la primera. */
//    stats.initial_report(cout, popSize, Gmax, Pc, Pm);

//    inicalizarPob();

//    evaluarPoblacion(oldpop);
//    elitismo(oldpop, gen);
//    stats.statistics(oldpop, popSize);

//    for (gen=2; gen <= Gmax; gen++) {
//       /* Seleccionar los padres guiados por la aptitud. */
//       seleccionPadres(oldpop);

//       /* Cruzar los pares de padres para producir la población de hijos. */
//       cruza(oldpop, newpop);

//       /* Mutar a los hijos según Pm. */
//       mutacion(newpop);

//       /* Evaluar la nueva generación */
//       evaluarPoblacion(newpop);
//       elitismo(newpop, gen); // Encontrar el mejor individuo.

//       /* Calcular las estadísticas sobre la aptitud en la nueva generación */
//       stats.statistics(newpop, popSize);

//       /* Imprimir los resultados de las estadísticas */
//       //stats.report(cout, oldpop, newpop, popSize, gen);  // Esto imprime TODA la información.
//       stats.shortReport(cout, oldpop, newpop, popSize, gen);

//       /* Ahora, la nueva generación será la vieja */
//       temp = oldpop;
//       oldpop = newpop;
//       newpop = temp;
//    }

//    // Para dejar las variables (PESOS) de la población final en este archivo.
//    // ***salidafinal*** debe estar donde corren este programa.
//    ofstream archVariables(outputDir + "/pesos_pob.txt", std::ofstream::out);

//    // Para dejar la evaluación (tiempo y distancia restante) de la población.
//    ofstream archEvaluacion(outputDir + "/eval_pob.txt", std::ofstream::out);

//    stats.writeVariables(archVariables, oldpop, popSize);
//    stats.writeEvaluation(archEvaluacion, oldpop, popSize);
//    archVariables.close();
//    archEvaluacion.close();
// }

// /* Creación aleatoria de la población inicial */
// void GeneticoSimple::inicalizarPob()
// {
//    for (int j=0; j < popSize; j++) {
//       oldpop[j].insuflar(problema, precision);
//       newpop[j].insuflar(problema, precision);
//    }

//    if (Pm == SIN_VALOR)
//       Pm = 1.0 / oldpop[0].chromoSize;
// }

// /* Evaluación de cada uno de los popsize individuos */
// void GeneticoSimple::evaluarPoblacion(Individuo* pop)
// {

//    for (int i = 0; i < popSize; ++i)
//    {
//       problema->evaluateFun( pop[i].x, pop[i].eval, pop[i].cons );

//       stats.nevals++;

//       /*** SIN restricción *****/
//       // eval: distancia para termina la carrera.
//       pop[i].aptitud = 1.0 / (1+pop[i].eval); // Para MINIMIZAR distancia por recorrer.

//       /*** Para considerar la restricción***/
//       // eval: tiempo en llegar a la meta.
//       // cons[0]: distancia que falta para llegar a la meta.

//       // Actualizar el tiempo máximo
// //       if (tiempoMaximo < pop[i].eval)  // tiempoMaximo se inicia en el constructor a 0.
// //          tiempoMaximo = pop[i].eval;
// //
// //       if ( pop[i].eval == 0.0 )  // el tiempo es 0, NO terminó.
// //          pop[i].aptitud = 1 / (pop[i].cons[0] + tiempoMaximo);
// //       else
// //          pop[i].aptitud = 1 / pop[i].eval;
//    }
// }

// /* Sección de los padres según su aptitud. */
// void GeneticoSimple::seleccionPadres(Individuo* pop) {
//    calcularValEsperado(pop);

//    for (int i = 0; i < popSize; i++)
//       padres[i] = seleccionRuleta(pop);
// }

// /* El algoritmo de selección proporcional de Ruleta */
// int GeneticoSimple::seleccionRuleta(Individuo* pop) {
//    double suma, ale;
//    int j;
//    uniform_real_distribution<> rdis(0.0, sumvalesp);

//    ale = rdis(rng);
//    suma = 0.0;
//    j = 0;

//    do {
//       suma += pop[j].valesp;
//       j++;
//    } while ( suma < ale  &&  j < popSize );

//    return j-1;
// }

// /* Crear la nueva generación por medio de selección, cruza y mutación */
// void GeneticoSimple::cruza(Individuo* oldpop, Individuo* newpop)
// {
//    int mate1, mate2, pcruza;

//    // En cada iteración cruzar dos padres y mutar los 2 hijos
//    for (int j = 0; j < popSize-1; j += 2) // <--- j avanza de 2 en 2.
//    {
//       mate1 = padres[j];   // índice del padre 1
//       mate2 = padres[j+1]; // índice del padre 2

//       /* Se efectúa la cruza para producir dos nuevos individuos */
//       pcruza = cruza1Punto(oldpop[mate1].cromo, oldpop[mate2].cromo,
//                            newpop[j].cromo, newpop[j+1].cromo);
//       newpop[j  ].xsite1 = pcruza;
//       newpop[j+1].xsite1 = pcruza;
//       newpop[j  ].padres[0] = mate1+1;
//       newpop[j  ].padres[1] = mate2+1;
//       newpop[j+1].padres[0] = mate1+1;
//       newpop[j+1].padres[1] = mate2+1;
//    }
// }

// /* El operador de cruza aplicado a un individuo dado */
// int GeneticoSimple::cruza1Punto(Cromosoma& padre1, Cromosoma& padre2,
//                                 Cromosoma& hijo1,  Cromosoma& hijo2)
// {
//    int pcruza, j;
//    int chromoSize = padre1.size();

//    /* Para elegir el punto de cruza entre 0 y chromoSize-1 */
//    uniform_int_distribution<int> unif(0, chromoSize - 1);

//    /* Realizar la cruza con una probabilidad Pc */
//    if( flip(Pc) ) {
//       pcruza = unif(rng);
//       /* Realizar la cruza */
//       for ( j = (chromoSize-1); j >= (chromoSize - pcruza); j-- ) {
//          hijo1[j] = padre1[j];
//          hijo2[j] = padre2[j];
//       }

//       for ( j = (chromoSize - pcruza)-1; j >= 0 ; j-- ) {
//          hijo1[j] = padre2[j];
//          hijo2[j] = padre1[j];
//       }

//       stats.ncruzas++; /* Guardar el número de cruzas */
//    }
//    else { /* los padres pasan idénticos a la siguiente generación */
//       hijo1 = padre1;
//       hijo2 = padre2;
//       pcruza = 0;
//    }

//    return pcruza;
// }

// void GeneticoSimple::mutacion(Individuo* pop)
// {
//    // En cada iteración cruzar dos padres y mutar los 2 hijos
//    for (int j = 0; j < popSize; j++) {
//       /* Mutar (si es el caso) los hijos resultantes de la cruza. */
//       pop[j].nMutaciones = mutacionUniforme(pop[j].cromo);
//       pop[j].decodificar();

//       if (pop[j].nMutaciones > 0)
//          stats.nmutaciones++; /* Solamente registra cuántos individuos mutaron. */
//    }

//    /* Mantener al mejor individuo colocándolo en una posición aleatoria */
//    uniform_int_distribution<int> unif(0, popSize - 1);
//    int ale = unif(rng);
//    pop[ale].copiar(&stats.bestfit);
//    stats.positionBestFit = ale+1;
// }

// /*  El operador de mutación aplicado a un individuo dado */
// int GeneticoSimple::mutacionUniforme(Cromosoma& cromo)
// {
//    int numMutations = 0;
//    /* Recorrer todos los bits y mutar si es necesario. */
//    for (unsigned k=0; k < cromo.size(); k++) {
//       if ( flip(Pm) ) {
//          numMutations++;
//          cromo[k] = (cromo[k] == 0) ? 1 : 0;
//       }
//    }

//    return numMutations;
// }

// void GeneticoSimple::calcularValEsperado(Individuo* pop)
// {
//    /* Sumar la aptitud */
//    double sumaptitud = 0.0;
//    for (int j=0; j < popSize; j++ )
//       sumaptitud = sumaptitud + pop[j].aptitud;

//    /* Calcular el promedio de la aptitud */
//    stats.avgApt = sumaptitud/popSize;

//    /* Calcular la suma del valor esperado de los individuos */
//    sumvalesp = 0.0; /* Este valor se usa en la selección por ruleta. */
//    for (int j=0; j < popSize; j++) {
//       if ( stats.avgApt != 0.0 )
//          pop[j].valesp = pop[j].aptitud / stats.avgApt;
//       else
//          pop[j].valesp = 0.0f;

//       sumvalesp += pop[j].valesp;
//    }
// }

// void GeneticoSimple::elitismo(Individuo* pop, int gen)
// {
//    for (int j=0; j < popSize; j++ )
//    {
//       /* Probar si se ha encontrado un nuevo máximo GLOBAL */
//       if (pop[j].aptitud > stats.bestfit.aptitud) {
//          stats.bestfit.copiar(&pop[j]);
//          stats.generationBestFit = gen;
//          stats.positionBestFit = j+1;
//       }
//    }
// }

// /* Lanzamiento de una moneda sesgada - true si cae cara */
// int GeneticoSimple::flip(double prob)
// {
//    uniform_real_distribution<> rdis(0.0, 1.0);

//    if( rdis(rng) <= prob)
//       return true;
//    else
//       return false;
// }
