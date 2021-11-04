 #include "ControllerParticle.hpp"
 #include "../AbstractAgent.hpp"
 #include "../Point.hpp"
 #include "../Goal.hpp"
 #include "../Robot.hpp"
 #include "../MathUtils.hpp"
 #include "RandomGenerator.hpp"
 namespace Model{
#define MAX_TRY 5

ControllerParticle::ControllerParticle(AbstractAgent * agent):AbstractFilterController(agent),filter(agent),pathPoint(0),command(std::make_shared<MoveActuatorCommand>(0,0)),determinePlaceCount(0)
{
    
	 	

}
ControllerParticle::~ControllerParticle()
{
    
}
void ControllerParticle::setCommand(double speed,double orientationInDegrees)
{
    command->angle = orientationInDegrees;
    command->distance = speed;
}
void ControllerParticle::ini()
{
    distributeParticles();
    determinePlaceCount = 0;
    pathPoint = 0;
    astarPath.clear();
    command = std::make_shared<MoveActuatorCommand>(0,0);
}
void ControllerParticle::distributeParticles()
{
    static RandomGenerator gen(0,1024);
    static RandomGenerator orien(0,359);
    Point tempP;
    for(size_t i = 0; i < NUMBEROFPARTICLES;++i)
    {
        tempP.x = gen.giveNextNumber();
        tempP.y = gen.giveNextNumber();
        believe_particles.push_back(Particle(tempP,orien.giveNextNumber()));
    }
}
void ControllerParticle::calculateNextAction()
{

        if(MAX_TRY == determinePlaceCount)
        {
             command = std::make_shared<MoveActuatorCommand>(0,0);
    		 Robot * robot = dynamic_cast<Robot*>(agent);
             pathPoint = 0;
             Point pos = this->believeToPoint(),pos2 = robot->getGoal()->getPosition();
	         astarPath = astar.search(pos,pos2,Size(50,50) * 1.5);
             ++determinePlaceCount;
        }
        else if(MAX_TRY > determinePlaceCount)
        {   

            for(uint8_t i = 0 ; i < 4 ; ++i)
            {   
                if(std::static_pointer_cast<LidarPercept>(agent->getSensor(SENSORINDEX::LIDAR)->getPerceptFor(agent->getSensor(SENSORINDEX::LIDAR)->getStimulus()))->distances[i * 45][0] > 150) 
                {
                    command = std::make_shared<MoveActuatorCommand>(i * 90,20);
                    i = 4;
                }
            }
            ++determinePlaceCount;
        }
        else
        {
                Point pos = this->believeToPoint();
            	static PathAlgorithm::Vertex vertex(0,0);
                if(pathPoint + 20 < astarPath.size())
                {

                    vertex = astarPath[pathPoint+=static_cast<int>(20)];
                    
                }else
                {
                    vertex = astarPath[astarPath.size()-1];
                }
	
                double distance = std::sqrt(std::pow(pos.x - vertex.x,2) + std::pow(pos.y - vertex.y,2));
                if(distance > 20)
                {
                    distance = 20;
                }
                double angle = std::atan2(vertex.y - pos.y,vertex.x - pos.x);
                command = std::make_shared<MoveActuatorCommand>(Utils::MathUtils::normaliseDegrees(Utils::MathUtils::toDegrees(angle)),distance);
        }
        std::shared_ptr<Actuator> motor = std::static_pointer_cast<Actuator>(agent->getActuator(0));
        motor->handleCommand(command); 
}
Particle& ControllerParticle::findHighestWeight()
{
    Particle& temp = believe_particles[0];
    for(Particle& particle : believe_particles)
    {
        if(temp.getWeight() < particle.getWeight())
        {
            temp = particle;
        }

    }
    return temp;
}
Point ControllerParticle::believeToPoint()
{    
    return findHighestWeight().getPositie();
}

void ControllerParticle::update()
{
    std::vector<std::shared_ptr<AbstractPercept>> measurements{ 
        agent->getSensor(SENSORINDEX::COMPAS)->getPerceptFor(agent->getSensor(SENSORINDEX::COMPAS)->getStimulus()),
        agent->getSensor(SENSORINDEX::LIDAR)->getPerceptFor(agent->getSensor(SENSORINDEX::LIDAR)->getStimulus())
    };
    filter.filter(believe_particles,command,measurements);
}
std::tuple<double,double> ControllerParticle::getCommand()
{
    return std::make_tuple(command->distance,command->angle);
}

 }