#include "CrowdAgent.h"

CrowdAgent::CrowdAgent(const std::unordered_map<std::string, std::vector<std::string>>& externalGraph) : graph(externalGraph) {
    // Obtener el nodo del agente usando el DEF name.
    initializeWaypoints();

    agentNode = getFromDef(getName());

    initialDestination = getClosestWaypoint(agentNode->getField("translation")->getSFVec3f(), currentDestinationName);
    currentDestination = initialDestination;

    #if DEBUG
    printConnectionGraph();

    std::cout << "Human: " << currentDestinationName << " Closest Waypoint: " << currentDestination << std::endl;  
    #endif
}

void CrowdAgent::checkArrival() {

    if(!currentDestination)
        return;


    // Aquí revisarías si el agente ha llegado a su destino.
    // Si ha llegado, entonces pides un nuevo destino al Crowd
    if (hasArrivedToDestination()) {

        currentDestination = getNextDestination(currentDestinationName);

        #if DEBUG
            std::cout << "[ " << getName() << "] " << "Destination arrived. New destination: " << currentDestinationName << std::endl;
        #endif
    }
}

void CrowdAgent::update(){
    moveToDestination();
    checkArrival(); 
}

void CrowdAgent::moveToDestination() {

    if(!agentNode)
        return;

    // Si no tiene destino asignado hacemos que no se mueva.
    if(!currentDestination){
        const double stopVelocity[] = {0, 0, 0, 0, 0, 0};
        agentNode->setVelocity(stopVelocity);
        return;
    }
        
    const double* currentPosition = agentNode->getField("translation")->getSFVec3f();
    // Calcular la dirección hacia el destino
    double velocity[3] = {
            currentDestination[0] - currentPosition[0],
            currentDestination[1] - currentPosition[1],
            0
    };

    // Normalizar el vector dirección
    double magnitude = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    if (magnitude > 0) {
        velocity[0] /= magnitude;
        velocity[1] /= magnitude;
    }

    // Calcular el vector de velocidad escalando el vector dirección por la velocidad deseada
    velocity[0] = velocity[0] * movementSpeed;
    velocity[1] = velocity[1] * movementSpeed;

    // Calcular el ángulo de rotación basado en el vector dirección
    double angle = atan2(velocity[1], velocity[0]);
    // Establecer la rotación del agente. Aquí asumimos que la rotación es alrededor del eje Z.
    double rotation[4] = {0, 0, 1, angle}; // {x, y, z, angle}

    agentNode->getField("rotation")->setSFRotation(rotation);
    agentNode->setVelocity(velocity);
}
    


// Retorna un nuevo destino basado en el destino actual.
const double* CrowdAgent::getNextDestination(std::string& currentDestinationDEF) {

    // Comprobamos que el elemento existe en el grafo
    if(!graph.count(currentDestinationDEF)){
        currentDestinationDEF = "";
        return nullptr;
    }

    auto connections = graph.find(currentDestinationDEF);

    if (connections->second.size() != 0) {
        // Genera un índice aleatorio entre 0 y connections->second.size() - 1
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dist(0, connections->second.size() - 1);
        int randomIndex = dist(gen);

        // Retorna el destino conectado en el índice aleatorio
        currentDestinationDEF = connections->second[randomIndex];

        return this->getFromDef(currentDestinationDEF)->getField("translation")->getSFVec3f();
    }

    return nullptr;
}

// Añadir conexión entre waypoints.
void CrowdAgent::addConnection(const std::string& from, const std::string& to) {
    graph[from].push_back(to);
}

bool CrowdAgent::hasArrivedToDestination() {

    if(!currentDestination)
        return false;

    const double* agentPosition = agentNode->getField("translation")->getSFVec3f();

    double distance = sqrt(
            pow(agentPosition[0] - currentDestination[0], 2) +
            pow(agentPosition[1] - currentDestination[1], 2)
    );

    return distance < 0.1;  // Suponiendo 0.1 como un umbral de llegada.
}

void CrowdAgent::initializeWaypoints() {
    // Almacenamos nodo padre
    webots::Node* waypointsNode = this->getFromDef("WAYPOINTS");
    if (!waypointsNode){
        std::cerr << "Group parent node with DEF 'WAYPOINTS' not found." << std::endl;
        return;
    }
        

    // Accedemos a los hijos del nodo padre
    webots::Field* childrenField = waypointsNode->getField("children");
    if (!childrenField || childrenField->getType() != webots::Field::MF_NODE){
        std::cerr << "Node with DEF 'WAYPOINTS' needs to be a Group node." << std::endl;
        return;
    }

    int numberOfNodes = childrenField->getCount();
    // Recorrer todos los nodos bajo WAYPOINTS y encontrar aquellos que son waypoints.
    for (int i = 0; i < numberOfNodes; i++) {

        webots::Node* childNode = childrenField->getMFNode(i);
        if(!childNode || childNode->getTypeName() != "Pose")
            continue;

        std::string nodeName = childNode->getDef();
        // Si su DEF comienza con el nombre WAYPOINT_
        if (nodeName.find("WAYPOINT_") != std::string::npos)
            // Lo almacenamos como waypoint
            waypoints[nodeName] = childNode;

    }
}

std::string CrowdAgent::Vector3toString(const double* vec) {
    if (vec == nullptr) {
        std::cerr << "Error: puntero nulo proporcionado." << std::endl;
        return std::string();
    }

    std::string stringVector = "[" + std::to_string(vec[0]) + ", " + std::to_string(vec[1]) + ", " + std::to_string(vec[2]) + "]";
    return stringVector;
}

// Función para obtener el waypoint más cercano a una posición dada.
const double* CrowdAgent::getClosestWaypoint(const double* position, std::string& currentDestinationName) {
    
    double minDistance = std::numeric_limits<double>::max();
    const double* closestWaypoint = nullptr;

    for (const auto& waypoint : waypoints) {
        const double* waypointPosition = waypoint.second->getField("translation")->getSFVec3f();

        #if DEBUG
            std::cout << waypoint.first << ": " << Vector3toString(waypointPosition) << std::endl;
        #endif

        // Calcula la distancia a cada waypoint en los ejes X e Y.
        double distance = sqrt(
                pow(position[0] - waypointPosition[0], 2) +
                pow(position[1] - waypointPosition[1], 2)
        );

        if (distance < minDistance) {
            minDistance = distance;
            closestWaypoint = waypointPosition;

            currentDestinationName = waypoint.first;
        }
    }

    return closestWaypoint;
}

// Método para imprimir las conexiones de un único waypoint
void CrowdAgent::printWaypointConnections(const std::string& waypointName) {
    auto it = graph.find(waypointName);
    if (it != graph.end()) {
        std::cout << waypointName << " está conectado con: ";
        for (const auto& connectedWaypoint : it->second) {
            std::cout << connectedWaypoint << " ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Waypoint " << waypointName << " no encontrado en el grafo." << std::endl;
    }
}

// Método para imprimir todos los waypoints y sus conexiones
void CrowdAgent::printConnectionGraph() {
    std::cout << std::endl << "------ CONNECTION GRAPH ------" << std::endl;

    for (const auto& pair : graph) {
        printWaypointConnections(pair.first);
    }


    std::cout << "------ ------ ------ ------" << std::endl << std::endl;
}

void CrowdAgent::reset(){
    agentNode->getField("translation")->setSFVec3f(initialDestination);
    getClosestWaypoint(agentNode->getField("translation")->getSFVec3f(), currentDestinationName);
    currentDestination = initialDestination;
}