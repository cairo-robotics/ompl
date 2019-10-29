
/* Author: Jack Kawell */

#include "ompl/base/objectives/CustomObjective.h"
#include "ompl/tools/config/MagicConstants.h"
#include <limits>

// ROS Client/Server
#include "ros/ros.h"
#include "ompl/CustomCost.h"
#include <cstdlib>

// Extracting state info
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <stdio.h>

ompl::base::CustomObjective::
CustomObjective(const SpaceInformationPtr &si) :
    MinimaxObjective(si)
{
    this->setCostThreshold(Cost(std::numeric_limits<double>::infinity()));
}

ompl::base::Cost ompl::base::CustomObjective::stateCost(const State *s) const
{
    // Pull out the model based state given by MoveIt!
    const ompl_interface::ModelBasedStateSpace::StateType *state = s->as<ompl_interface::ModelBasedStateSpace::StateType>();

    // Print the state
    printf("State: ");
    unsigned int dimension = si_->getStateSpace()->getDimension();
    for (unsigned int i = 0; i < dimension; ++i)
        printf("%f ", state->values[i]);
    printf("\n");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ompl::CustomCost>("custom_cost");
    ompl::CustomCost srv;
    srv.request.x = 1.0;
    float costValue;
    if (client.call(srv))
    {
        costValue = srv.response.cost;
    }
    else
    {
        ROS_ERROR("Failed to call service custom_cost. Using clearance instead.");
        costValue = si_->getStateValidityChecker()->clearance(s);
    }
    
    return Cost(costValue);
}

bool ompl::base::CustomObjective::isCostBetterThan(Cost c1, Cost c2) const
{
    return c1.value() > c2.value();
}

ompl::base::Cost ompl::base::CustomObjective::identityCost() const
{
    return Cost(std::numeric_limits<double>::infinity());
}

ompl::base::Cost ompl::base::CustomObjective::infiniteCost() const
{
    return Cost(-std::numeric_limits<double>::infinity());
}
