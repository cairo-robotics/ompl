
/* Author: Jack Kawell */

#ifndef OMPL_BASE_OBJECTIVES_CUSTOM_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_CUSTOM_OBJECTIVE_

#include "ompl/base/objectives/MinimaxObjective.h"

namespace ompl
{
    namespace base
    {
        /** \brief Objective for attempting to maximize the minimum clearance along a path. */
        class CustomObjective : public MinimaxObjective
        {
        public:
            CustomObjective(const SpaceInformationPtr &si);

            /** \brief Defined as the clearance of the state \e s, which is computed using the StateValidityChecker in this objective's SpaceInformation */
            virtual Cost stateCost(const State *s) const;

            /** \brief Since we wish to maximize clearance, and costs are equivalent to path clearance, we return the greater of the two cost values. */
            virtual bool isCostBetterThan(Cost c1, Cost c2) const;

            /** \brief Returns +infinity, since any cost combined with +infinity under this objective will always return the other cost. */
            virtual Cost identityCost() const;

            /** \brief Returns -infinity, since no path clearance value can be considered worse than this. */
            virtual Cost infiniteCost() const;
        };
    }
}

#endif
