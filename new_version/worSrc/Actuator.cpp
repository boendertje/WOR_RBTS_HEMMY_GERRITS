    #include "Actuator.hpp"
    #include "../Robot.hpp"
    #include "../MathUtils.hpp"
    #include "MainApplication.hpp"
    #include "Shape2DUtils.hpp"
    namespace Model
    {
    void Actuator::handleCommand(std::shared_ptr<AbstractCommand> anAbstractCommand)
    {
        Robot* robot = dynamic_cast<Robot*>(agent);
        std::tuple<Point,double> newPosition;
        MoveActuatorCommand* command = dynamic_cast< MoveActuatorCommand* >(anAbstractCommand.get());
        Point position = robot->getPosition();
        newPosition = drive(position,command->distance,command->angle);

        robot->setFront(BoundedVector(std::get<0>(newPosition),robot->getPosition()));
        robot->setPosition(std::get<0>(newPosition));
    }
    Actuator::Actuator():AbstractActuator(nullptr),stddev(0),orientation(0)
    {
    }
    Actuator::Actuator(AbstractAgent* agent,double stddev):AbstractActuator(agent),stddev(stddev),orientation(0)
    {
    }
    Actuator::~Actuator()
    {

    }
    std::tuple<Point,double> Actuator::drive(Point& position,double speed, double orientationInDegrees)
    {
        orientation = orientationInDegrees = Utils::MathUtils::normaliseDegrees(orientationInDegrees);
        static RandomSTD noise(0,stddev);
        static RandomSTD noise2(0,stddev);
        position.x += static_cast<int>(std::round(speed * cos(Utils::MathUtils::toRadians(orientationInDegrees)) + noise.giveNextNumber()));
        position.y += static_cast<int>(std::round(speed * sin(Utils::MathUtils::toRadians(orientationInDegrees)) + noise2.giveNextNumber()));
        return std::make_tuple(position,orientationInDegrees);
    }
    }