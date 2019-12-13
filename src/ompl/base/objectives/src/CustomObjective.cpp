
/* Author: Jack Kawell */

#include "ompl/base/objectives/CustomObjective.h"
#include "ompl/tools/config/MagicConstants.h"
#include <limits>

// ROS Client/Server
#include "ros/ros.h"
#include "moveit_msgs/CustomCost.h"
#include <cstdlib>

// Extracting state info
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>

ompl::base::CustomObjective::
CustomObjective(const SpaceInformationPtr &si) :
    MinimaxObjective(si)
{
    this->setCostThreshold(Cost(std::numeric_limits<double>::infinity()));
}

ompl::base::Cost ompl::base::CustomObjective::stateCost(const State *s) const
{
    // Static local variable to make sure to only print warning once
    static bool printWarning = true;

    // Create ROS client
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<moveit_msgs::CustomCost>("custom_cost");
    moveit_msgs::CustomCost srv;

    // Pull out the model based state given by MoveIt!
    // NOTE: This asumes a ModelBasedStateSpace is used. May not be the case on all systems!!
    const ompl_interface::ModelBasedStateSpace::StateType *state = s->as<ompl_interface::ModelBasedStateSpace::StateType>();
    unsigned int dimension = si_->getStateSpace()->getDimension();

    // Build the service request
    srv.request.state.clear();
    for (unsigned int i = 0; i < dimension; ++i)
    {
        srv.request.state.push_back(state->values[i]);
    }

    // Request cost from ROS server (if call successful use that cost, else use clearance)
    float costValue;
    if (client.call(srv))
    {
        costValue = srv.response.cost;
        // Print warning again if connection to server is lost
        printWarning = true;
    }
    else
    {
        // Only print the warning once
        if (printWarning)
        {
            ROS_WARN("Failed to call service custom_cost. Using clearance instead.");
            printWarning = false;
        }
        costValue = si_->getStateValidityChecker()->clearance(s);
    }

    // Return cost as OMPL Cost object
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
