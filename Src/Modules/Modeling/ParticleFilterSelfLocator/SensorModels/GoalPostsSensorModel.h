/**
* @file GoalPostsSensorModel.h
*
* Sensor model for updating samples by perceived goal posts
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once


/**
* @class GoalPostsSensorModel
*/
class GoalPostsSensorModel: public SensorModel
{
private:
  /** Reference to goal percept which contains goal post information */
  const GoalPercept& theGoalPercept;

public:
  /** Constructor. */
  GoalPostsSensorModel(const SelfLocatorParameter& selfLocatorParameter,
                       const GoalPercept& goalPercept, const FrameInfo& frameInfo,
                       const FieldDimensions& fieldDimensions, const CameraMatrix& cameraMatrix,
                       const PerceptValidityChecker& perceptValidityChecker):
    SensorModel(selfLocatorParameter, frameInfo, fieldDimensions, cameraMatrix,
                perceptValidityChecker, Observation::GOAL_POST),
    theGoalPercept(goalPercept)
  {}

  /** Function for computing weightings for a sample set.
  * @param samples The samples (not changed by this function
  * @param selectedIndices The indices of the selected observations.
  * @param weightings List of weightings. -1 means: no update
  * @return An overall result of the computation
  */
  SensorModelResult computeWeightings(const SampleSet<SelfLocatorSample>& samples,
                                      const vector<int>& selectedIndices, vector<float>& weightings)
  {
    bool updated(false);
    const float& camZ = theCameraMatrix.translation.z;
    if(selectedIndices[0] < GoalPercept::NUMBER_OF_GOAL_POSTS)
    {
      // Identified (left or right) goal posts
      for(vector<int>::const_iterator i = selectedIndices.begin(); i != selectedIndices.end(); ++i)
      {
        int p = *i;
        const GoalPost& post = theGoalPercept.posts[p];
        // Precompute stuff:
        const float distanceStdDev = (post.distanceType == GoalPost::HEIGHT_BASED) ?
                                     theSelfLocatorParameter.standardDeviationGoalpostSizeDistance : theSelfLocatorParameter.standardDeviationGoalpostBearingDistance;
        const float bestPossibleAngleWeighting    = gaussianProbability(0.0f, theSelfLocatorParameter.standardDeviationGoalpostAngle);
        const float bestPossibleDistanceWeighting = gaussianProbability(0.0f, distanceStdDev);
        const Vector2<> pField((float) post.positionOnField.x, (float) post.positionOnField.y);
        const float angleObserved = pField.angle();
        const float distanceAsAngleObserved = (pi_2 - atan2(camZ, pField.abs()));
        Vector2<> uniquePosition;
        if(p == GoalPercept::LEFT_OPPONENT)
          uniquePosition = Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosLeftGoal);
        else if(p == GoalPercept::RIGHT_OPPONENT)
          uniquePosition = Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosRightGoal);
        else if(p == GoalPercept::LEFT_OWN)
          uniquePosition = Vector2<>((float) theFieldDimensions.xPosOwnGoalpost, (float) theFieldDimensions.yPosRightGoal);
        else if(p == GoalPercept::RIGHT_OWN)
          uniquePosition = Vector2<>((float) theFieldDimensions.xPosOwnGoalpost, (float) theFieldDimensions.yPosLeftGoal);
        // Iterate over all samples and compute weightings:
        for(int i = 0; i < samples.size(); ++i)
        {
          const SelfLocatorSample& s(samples.at(i));
          const Pose2D sPose(s.angle, (float) s.translation.x, (float) s.translation.y);
          if(updated)
            weightings[i] *= computeAngleWeighting(angleObserved, uniquePosition, sPose,
                                                   theSelfLocatorParameter.standardDeviationGoalpostAngle, bestPossibleAngleWeighting);
          else
            weightings[i] = computeAngleWeighting(angleObserved, uniquePosition, sPose,
                                                  theSelfLocatorParameter.standardDeviationGoalpostAngle, bestPossibleAngleWeighting);
          if(post.distanceType != GoalPost::IS_CLOSER)
          {
            weightings[i] *= computeDistanceWeighting(distanceAsAngleObserved, uniquePosition,
                             sPose, camZ, distanceStdDev, bestPossibleDistanceWeighting);
          }
        }
        updated = true;
      }
    }
    else
    {
      // Use unknown posts only, if there has not been an update by a "normal" post
      for(vector<int>::const_iterator i = selectedIndices.begin(); i != selectedIndices.end(); ++i)
      {
        int p = *i - GoalPercept::NUMBER_OF_GOAL_POSTS;
        const GoalPost& post = theGoalPercept.unknownPosts[p];
        const float distanceStdDev = (post.distanceType == GoalPost::HEIGHT_BASED) ?
                                     theSelfLocatorParameter.standardDeviationGoalpostSizeDistance : theSelfLocatorParameter.standardDeviationGoalpostBearingDistance;
        const float bestPossibleAngleWeighting    = gaussianProbability(0.0f, theSelfLocatorParameter.standardDeviationGoalpostAngle);
        const float bestPossibleDistanceWeighting = gaussianProbability(0.0f, distanceStdDev);
        const Vector2<> pField((float) post.positionOnField.x, (float) post.positionOnField.y);
        const float distanceAsAngleObserved = (pi_2 - atan2(camZ, pField.abs()));
        const float angleObserved = pField.angle();
        Vector2<> uniquePositions[2];
        if(p == GoalPercept::UNKNOWN_OPPONENT)
        {
          uniquePositions[0] = Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosLeftGoal);
          uniquePositions[1] = Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosRightGoal);
        }
        else
        {
          uniquePositions[0] = Vector2<>((float) theFieldDimensions.xPosOwnGoalpost, (float) theFieldDimensions.yPosRightGoal);
          uniquePositions[1] = Vector2<>((float) theFieldDimensions.xPosOwnGoalpost, (float) theFieldDimensions.yPosLeftGoal);
        }
        // Iterate over all samples and compute weightings:
        for(int i = 0; i < samples.size(); ++i)
        {
          const SelfLocatorSample& s(samples.at(i));
          const Pose2D sPose(s.angle, (float) s.translation.x, (float) s.translation.y);
          float weighting0 = computeAngleWeighting(angleObserved,
                             uniquePositions[0], sPose, theSelfLocatorParameter.standardDeviationGoalpostAngle,
                             bestPossibleAngleWeighting);
          float weighting1 = computeAngleWeighting(angleObserved,
                             uniquePositions[1], sPose, theSelfLocatorParameter.standardDeviationGoalpostAngle,
                             bestPossibleAngleWeighting);
          if(post.distanceType != GoalPost::IS_CLOSER)
          {
            weighting0 *= computeDistanceWeighting(distanceAsAngleObserved, uniquePositions[0],
                                                   sPose, camZ, distanceStdDev, bestPossibleDistanceWeighting);
            weighting1 *= computeDistanceWeighting(distanceAsAngleObserved, uniquePositions[1],
                                                   sPose, camZ, distanceStdDev, bestPossibleDistanceWeighting);
          }
          if(updated)
            weightings[i] *= max(weighting0, weighting1);
          else
            weightings[i] = max(weighting0, weighting1);
        }
        updated = true;
      }
    }
    return FULL_SENSOR_UPDATE;
  }
};
