/**
* @file ObstacleModel.h
*
* Declaration of class ObstacleModel
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Math/Matrix2x2.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Enum.h"


/**
* @class ObstacleModel
*
* A class that represents the current state of the robot's environment
*/
class ObstacleModel : public Streamable
{
public:
  /** A single obstacle */
  class Obstacle : public Streamable
  {
  public:
    Vector2<> leftCorner;      /**< Leftmost point of the obstacle */
    Vector2<> rightCorner;     /**< Rightmost point of the obstacle */
    Vector2<> center;          /**< Center of mass of obstacle */
    Vector2<> closestPoint;    /**< Point of obstacle that is closest to the robot */
    int size;                  /**< The number of cells of the obstacle */
    Matrix2x2<> covariance;
    ENUM(Type, US, ROBOT, ARM);/**< Different obstacle type */
    Type type;                 /**< The type of an obstacle */

    /** Empty default constructor*/
    Obstacle() {}

    /** Constructor */
    Obstacle(const Vector2<>& leftCorner, const Vector2<>& rightCorner,
             const Vector2<>& center, const Vector2<>& closestPoint, int size, const Matrix2x2<>& covariance, Type type = US) :
      leftCorner(leftCorner), rightCorner(rightCorner), center(center), closestPoint(closestPoint), size(size), covariance(covariance), type(type) {}

  private:
    /**
    * Makes the object streamable
    * @param in The stream from which the object is read
    * @param out The stream to which the object is written
    */
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(leftCorner);
      STREAM(rightCorner);
      STREAM(center);
      STREAM(closestPoint);
      STREAM(size);
      STREAM(covariance);
      STREAM(type);
      STREAM_REGISTER_FINISH();
    }
  };

  /** A list of obstacles */
  std::vector<Obstacle> obstacles;

  /** Function for drawing */
  void draw();

private:
  /** Streaming function
  * @param in Object for streaming in the one direction
  * @param out Object for streaming in the other direction
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(obstacles);
    STREAM_REGISTER_FINISH();
  }
};
