/**
* @file XabslState.cpp
*
* Implementation of class State and helper classes
*
* @author <a href="http://www.sim.informatik.tu-darmstadt.de/pers/card/risler.html">Max Risler</a>
*/

#include "XabslState.h"
#include "XabslOption.h"

namespace xabsl
{

State* TransitionToState::getNextState()
{
  return nextState;
}

State::State(const char* name, ErrorHandler& errorHandler,
             const unsigned& time,
             int optionIndex,
             int index)
  : NamedItem(name),
    index(index),
    optionIndex(optionIndex),
    targetState(false),
    errorHandler(errorHandler),
    decisionTree(0),
    time(time)
{
}

State::~State()
{
  if(decisionTree != 0) delete decisionTree;
  for(int i = 0; i < actions.getSize(); i++)
    delete actions[i];
}

void State::create(InputSource& input,
                   NamedArray<Option*>& options,
                   NamedArray<BasicBehavior*>& basicBehaviors,
                   NamedArray<State*>& states,
                   Symbols& symbols)
{
  XABSL_DEBUG_INIT(errorHandler.message("creating state \"%s\"", n));

  char c[100];
  int i;

  // target state or not
  input.readString(c, 1);
  if(*c == '1')
  {
    targetState = true;
  }

  // subsequent option, basic behaviors and output symbol assignments
  int numberOfActions = (int)input.readValue();

  subsequentOption = 0;
  for(i = 0; i < numberOfActions; i++)
  {
    Action* action =
      Action::create(input,
                     options,
                     basicBehaviors,
                     symbols,
                     *options[optionIndex],
                     *this,
                     errorHandler,
                     time);
    if(errorHandler.errorsOccurred)
    {
      errorHandler.error("XabslState::create(): could not create action for state \"%s\"", n);
      return;
    }
    if(subsequentOption == 0)
      if(ActionOption* actionOption = dynamic_cast<ActionOption*>(action))
        subsequentOption = actionOption->option;
    actions.append(action);
  }

  // transition to state or if / else block
  decisionTree = Statement::createStatement(input, errorHandler, symbols, *options[optionIndex], *this);
  if(errorHandler.errorsOccurred)
    errorHandler.error("XabslState::create(): could not create decision tree for state \"%s\"", n);
}

State* State::getNextState()
{
  timeOfStateExecution = time - timeWhenStateWasActivated;

  State* nextState = decisionTree->getNextState();

  return nextState->coopCheck() ? nextState : this;
}

void State::reset()
{
  timeWhenStateWasActivated = time;
  timeOfStateExecution = 0;
}

bool State::isTargetState() const
{
  return targetState;
}

} // namespace

