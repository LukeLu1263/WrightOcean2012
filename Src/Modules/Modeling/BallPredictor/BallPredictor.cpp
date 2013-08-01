/**
* @file BallPredictor.cpp
* The Implementation of the module that predicts the visibility or the position of the ball.
* @author Colin Graf
*/

#include "BallPredictor.h"
#include "Tools/Team.h"

MAKE_MODULE(BallPredictor, Modeling)

BallPredictor::BallPredictor() : ballWasSeen(false), lastBallShouldBeVisible(false), timeWhenShouldHaveAppeared(0) {}

void BallPredictor::update(BallHypotheses& ballHypotheses)
{
  bool ballCantBeVisible = abs(theBallModel.estimate.position.angle()) >= fromDegrees(85);
  bool ballShouldBeVisible = false;
  {
    const Vector3<> camera = theCameraMatrix.invert() * Vector3<>(theBallModel.estimate.position.x, theBallModel.estimate.position.y, 0.f);
    if(camera.x > 1)
    {
      const float scale = -theCameraInfo.focalLength / camera.x;
      Vector2<> image(theCameraInfo.opticalCenter.x + scale * camera.y, theCameraInfo.opticalCenter.y + scale * camera.z);
      image = theImageCoordinateSystem.fromCorrectedApprox(image);
      if((image - Vector2<>(theCameraInfo.opticalCenter.x, theCameraInfo.opticalCenter.y)).squareAbs() < 100.f * 100.f)
        ballShouldBeVisible = true;
    }
  }

  if(ballShouldBeVisible && !lastBallShouldBeVisible)
    timeWhenShouldHaveAppeared = theFrameInfo.time;

  if(ballShouldBeVisible && theFrameInfo.getTimeSince(timeWhenShouldHaveAppeared) > 200 && theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > 200)
    ballWasSeen = false; // the ball is not seen although it should be visible
  if(theBallModel.timeWhenLastSeen == theFrameInfo.time)
    ballWasSeen = true;

  if(theBallModel.timeWhenLastSeen == theFrameInfo.time)
    ballHypotheses.timeWhenDisappeared = theFrameInfo.time;
  if(!ballShouldBeVisible && ballWasSeen && !ballCantBeVisible && theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 10000)
    ballHypotheses.timeWhenDisappeared = theFrameInfo.time;

  //
  lastBallShouldBeVisible = ballShouldBeVisible;
  TEAM_OUTPUT_FAST(idTeamMateBallHypotheses, bin, ballHypotheses);
}
