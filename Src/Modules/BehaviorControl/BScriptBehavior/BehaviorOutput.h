#pragma once

#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Infrastructure/SoundRequest.h"
#include "Output/BSWalkTo.h"
#include "Output/BSBikeDynPoints.h"
#include "Output/LEDsOut.h"
#include "Input/GameInfoWrapper.h"
#include "Input/RobotInfoWrapper.h"
#include "Input/TeamInfoWrapper.h"

struct BehaviorOutput
{
  void init(const InputRepresentations& inputRepresentations)
  {
    bsWalkTo = new BSWalkTo(inputRepresentations, &motionRequest);
    bsBikeDynPoints = new BSBikeDynPoints(&motionRequest, inputRepresentations);
    ledsOut.init(behaviorLEDRequest);
  }

  void reset(const InputRepresentations& inputReps)
  {
    /* this causes the robot to stand up if the engine can not be initialized
    //stand
    motionRequest.motion = MotionRequest::stand;
    */

    //look down
    headMotionRequest.mode = HeadMotionRequest::panTiltMode;
    headMotionRequest.cameraControlMode = HeadMotionRequest::lowerCamera;
    headMotionRequest.pan = 0.0f;
    headMotionRequest.tilt = -5.0f;

    //reset BehaviorLEDRequest
    behaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::default_color;
    behaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::default_color;
    for(int i = 0; i < BehaviorLEDRequest::numOfBehaviorLEDs; ++i)
      behaviorLEDRequest.modifiers[i] = LEDRequest::on;

    //sound
    soundRequest.sound = SoundRequest::none;

    //gamestate, penalty, kickoff, team
    gameInfo.update(inputReps.theGameInfo);
    robotInfo.update(inputReps.theRobotInfo);
    teamInfo.update(inputReps.theOwnTeamInfo, inputReps.theOpponentTeamInfo);
  }

  MotionRequest         motionRequest;
  HeadMotionRequest     headMotionRequest;
  SoundRequest          soundRequest;
  BehaviorLEDRequest    behaviorLEDRequest;
  BSWalkTo*             bsWalkTo;
  BSBikeDynPoints*      bsBikeDynPoints;
  LEDsOut               ledsOut;
  GameInfoWrapper       gameInfo;
  RobotInfoWrapper      robotInfo;
  TeamInfoWrapper       teamInfo;

  //ugly shice
  //TODO: !!!! Refactor stuff until the following functions disappear !!!
  void walkTo(float x, float y, float theta, float speed, bool rough)
  {
    bsWalkTo->walkTo(Pose2D(theta, x, y), speed, rough);
  }

  void setBikeDynPoints(int bmotion, const Vector2<> &ball, bool mirror)
  {
    bsBikeDynPoints->set((BikeRequest::BMotionID)bmotion, ball, mirror);
  }
};

