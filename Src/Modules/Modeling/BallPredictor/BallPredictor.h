/**
* @file BallPredictor.h
* Declares a module that predicts the visibility or the position of the ball.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/BallHypotheses.h"

MODULE(BallPredictor)
  REQUIRES(CameraMatrix)
  REQUIRES(FieldDimensions)
  REQUIRES(CameraInfo)
  REQUIRES(FrameInfo)
  REQUIRES(TeamMateData) // only for TEAM_OUTPUT_FAST
  REQUIRES(BallModel)
  REQUIRES(ImageCoordinateSystem)
  PROVIDES_WITH_MODIFY(BallHypotheses)
END_MODULE

/**
* @class BallPredictor
* A modules that predicts the visibility or the position of the ball.
*/
class BallPredictor : public BallPredictorBase
{
public:
  /**
  * Default constructor.
  */
  BallPredictor();

private:
  bool ballWasSeen; /**< Whether the ball was seen when it should be visible */
  bool lastBallShouldBeVisible; /**< Whether the ball should have been visible in the previous iteration */
  unsigned int timeWhenShouldHaveAppeared; /**< The time when the ball should have appeard in image */

  /**
  * Updates the ball gypotheses provided from this module.
  * @param ballHypotheses The ballHypotheses updated from the module
  */
  void update(BallHypotheses& ballHypotheses);
};
