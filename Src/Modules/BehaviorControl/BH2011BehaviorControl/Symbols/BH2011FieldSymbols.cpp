/**
* @file BH2011FieldSymbols.cpp
*
* Implementation of class BH2011FieldSymbols.
*/

#include "BH2011FieldSymbols.h"


void BH2011FieldSymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerDecimalInputSymbol("field.ball_radius", &ballRadius);
  engine.registerDecimalInputSymbol("field.own_goal.x", &xPosOwnGoal);
  engine.registerDecimalInputSymbol("field.own_ground_line.x", &xPosOwnGroundline);
  engine.registerDecimalInputSymbol("field.opponent_ground_line.x", &xPosOpponentGroundline);
  engine.registerDecimalInputSymbol("field.left_sideline.y", &yPosLeftSideline);
  engine.registerDecimalInputSymbol("field.right_sideline.y", &yPosRightSideline);
  engine.registerDecimalInputSymbol("field.left_field_border.y", &yPosLeftFieldBorder);
  engine.registerDecimalInputSymbol("field.right_field_border.y", &yPosRightFieldBorder);
  engine.registerDecimalInputSymbol("field.center_circle_radius", &centerCircleRadius);
  engine.registerDecimalInputSymbol("field.own_penalty_area.x", &xPosOwnPenaltyArea);
  engine.registerDecimalInputSymbol("field.own_penalty_mark.x", &xPosOwnPenaltyMark);
  engine.registerDecimalInputSymbol("field.opponent_penalty_area.x", &xPosOpponentPenaltyArea);
  engine.registerDecimalInputSymbol("field.opponent_penalty_mark.x", &xPosOpponentPenaltyMark);
  engine.registerDecimalInputSymbol("field.left_penalty_area.y", &yPosLeftPenaltyArea);
  engine.registerDecimalInputSymbol("field.right_penalty_area.y", &yPosRightPenaltyArea);
  engine.registerDecimalInputSymbol("field.opponent_throw_in_point.x", &xPosThrowInPointOpponentHalf);
  engine.registerDecimalInputSymbol("field.own_throw_in_point.x", &xPosThrowInPointOwnHalf);
  engine.registerDecimalInputSymbol("field.left_goal.y", &yPosLeftGoal);
  engine.registerDecimalInputSymbol("field.right_goal.y", &yPosRightGoal);
}
