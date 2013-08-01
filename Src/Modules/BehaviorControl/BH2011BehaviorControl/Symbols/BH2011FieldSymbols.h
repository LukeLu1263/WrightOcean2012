/**
* @file BH2011FieldSymbols.h
*
* Declaration of class BH2011FieldSymbols.
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/Configuration/FieldDimensions.h"


/**
* The Xabsl symbols that are defined in "field_symbols.xabsl"
*/
class BH2011FieldSymbols : public Symbols
{
public:
  /* Constructor */
  BH2011FieldSymbols(FieldDimensions const& fieldDimensions) : fieldDimensions(fieldDimensions)
  {}

  /** registers the symbols at the engine */
  void registerSymbols(xabsl::Engine& engine);

  /** init function, called after construction, before first use of any symbol */
  void init()
  {
    ballRadius = (float)fieldDimensions.ballRadius;
    xPosOwnGoal = (float)fieldDimensions.xPosOwnGoal;
    xPosOwnGroundline = (float)fieldDimensions.xPosOwnGroundline;
    xPosOpponentGroundline = (float)fieldDimensions.xPosOpponentGroundline;
    yPosLeftSideline = (float)fieldDimensions.yPosLeftSideline;
    yPosRightSideline = (float)fieldDimensions.yPosRightSideline;
    yPosLeftFieldBorder = (float)fieldDimensions.yPosLeftFieldBorder;
    yPosRightFieldBorder = (float)fieldDimensions.yPosRightFieldBorder;
    centerCircleRadius = (float)fieldDimensions.centerCircleRadius;
    xPosOwnPenaltyArea = (float)fieldDimensions.xPosOwnPenaltyArea;
    xPosOwnPenaltyMark = (float)fieldDimensions.xPosOwnPenaltyMark;
    xPosOpponentPenaltyArea = (float)fieldDimensions.xPosOpponentPenaltyArea;
    xPosOpponentPenaltyMark = (float)fieldDimensions.xPosOpponentPenaltyMark;
    yPosLeftPenaltyArea = (float)fieldDimensions.yPosLeftPenaltyArea;
    yPosRightPenaltyArea = (float)fieldDimensions.yPosRightPenaltyArea;
    xPosThrowInPointOwnHalf = (float)fieldDimensions.xPosThrowInPointOwnHalf;
    xPosThrowInPointOpponentHalf = (float)fieldDimensions.xPosThrowInPointOpponentHalf;
    yPosLeftGoal = (float) fieldDimensions.yPosLeftGoal;
    yPosRightGoal = (float) fieldDimensions.yPosRightGoal;
  };

private:
  FieldDimensions const& fieldDimensions;
  float ballRadius;
  float xPosOwnGoal;
  float xPosOwnGroundline;
  float xPosOpponentGroundline;
  float yPosLeftSideline;
  float yPosRightSideline;
  float yPosLeftFieldBorder;
  float yPosRightFieldBorder;
  float centerCircleRadius;
  float xPosOwnPenaltyArea;
  float xPosOwnPenaltyMark;
  float xPosOpponentPenaltyArea;
  float xPosOpponentPenaltyMark;
  float yPosLeftPenaltyArea;
  float yPosRightPenaltyArea;
  float xPosThrowInPointOwnHalf;
  float xPosThrowInPointOpponentHalf;
  float yPosLeftGoal;
  float yPosRightGoal;
};
