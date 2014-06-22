#include "ompl/control/PathControl.h"
#include "ompl/control/planners/ltl/LTLProblemDefinition.h"
#include "ompl/control/planners/ltl/LTLSpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

oc::LTLProblemDefinition::LTLProblemDefinition(const LTLSpaceInformationPtr& ltlsi)
    : ob::ProblemDefinition(ltlsi), ltlsi_(ltlsi)
{
    createGoal();
}

void oc::LTLProblemDefinition::addLowerStartState(const ob::State* s)
{
    ob::ScopedState<ob::CompoundStateSpace> fullStart(si_);
    ltlsi_->getFullState(s, fullStart.get());
    addStartState(fullStart);
}

ob::PathPtr oc::LTLProblemDefinition::getLowerSolutionPath(void) const
{
    PathControl* fullPath = static_cast<PathControl*>(getSolutionPath().get());
    ob::PathPtr lowPathPtr(new PathControl(ltlsi_->getLowSpace()));
    PathControl* lowPath = static_cast<PathControl*>(lowPathPtr.get());
    lowPath->getControls().assign(fullPath->getControls().begin(), fullPath->getControls().end());
    lowPath->getControlDurations().assign(
        fullPath->getControlDurations().begin(), fullPath->getControlDurations().end());
    for (std::size_t i = 0; i < fullPath->getStateCount(); ++i)
    {
        lowPath->append(ltlsi_->getLowSpace()->allocState());
        ltlsi_->getLowSpace()->copyState(lowPath->getState(i),
            ltlsi_->getLowLevelState(fullPath->getState(i)));
    }
    return lowPathPtr;
}

void oc::LTLProblemDefinition::createGoal(void)
{
    class LTLGoal : public base::Goal
    {
    public:
        LTLGoal(const LTLSpaceInformationPtr& ltlsi)
            : ob::Goal(ltlsi), ltlsi_(ltlsi), prod_(ltlsi->getProductGraph()) {}
        virtual ~LTLGoal(void) {}
        virtual bool isSatisfied(const ob::State* s) const
        {
            return prod_->isSolution(ltlsi_->getProdGraphState(s));
        }
    protected:
        const LTLSpaceInformationPtr ltlsi_;
        const ProductGraphPtr prod_;
    };

    // Some compilers have trouble with LTLGoal being hidden in this function,
    // and so we explicitly cast it to its base type.
    setGoal(ob::GoalPtr(static_cast<ob::Goal*>(new LTLGoal(ltlsi_))));
}
