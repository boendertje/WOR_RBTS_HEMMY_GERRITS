#include "AbstractController.hpp"
namespace Model{
AbstractFilterController::AbstractFilterController():agent(nullptr)
{
    
}
AbstractFilterController::AbstractFilterController(AbstractAgent* agent):agent(agent)
{
}

AbstractFilterController::~AbstractFilterController()
{
}

}