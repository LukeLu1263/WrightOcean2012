/**
* @file Tools/ImageProcessing/BresenhamLineScan.h
*
* Utility class which performs the Bresenham algorithm for line scanning.
* Some constructors set numberOfPixels which can be used as a termination condition to
* prevent scans outside the image boundaries.
*
* @author <a href="mailto:timlaue@tzi.de">Tim Laue</a>
* @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
* @author <a href="mailto:oberlies@sim.tu-darmstadt.de">Tobias Oberlies</a> (revised constructors and commenting)
*/

#pragma once

#include "Tools/Math/Geometry.h"


class BresenhamLineScan
{
public:
  /**
   * Constructs a scanline through the two points. numberOfPixels can be used.
   * @param start The start point of the line segment.
   * @param end The end point of the line segment.
   */
  BresenhamLineScan(const Vector2<int>& start, const Vector2<int>& end);

  /**
   * Constructs a scanline with the given direction.
   * @param direction The direction vector of the scanline.
   */
  BresenhamLineScan(const Vector2<>& direction);

  /**
   * Constructs a scanline with the given direction.
   * @param direction The direction (angle) of the line, expressed in radians.
   */
  BresenhamLineScan(const float& direction);

  /**
   * Constructs a scanline with the given direction starting at start and ending at the
   * image boundary. numberOfPixels can be used.
   * @param start The start point of the line.
   * @param direction The direction (angle) of the line, expressed in radians.
   * @param cameraInfo The cameraInfo object of the camera that captured the image.
   */
  BresenhamLineScan(const Vector2<int>& start, const float& direction, const CameraInfo& cameraInfo);

  /**
   * Constructs a scanline with the given direction starting at start and ending at the
   * image boundary. numberOfPixels can be used.
   * @param start The start point of the ray
   * @param direction The vector pointing in the direction of scanning. Must not be a null
   *        vector.
   * @param cameraInfo The cameraInfo object of the camera that captured the image.
   * @author Tobias Oberlies
   */
  BresenhamLineScan(const Vector2<int>& start, const Vector2<>& direction, const CameraInfo& cameraInfo);



  /** Resets the error counter */
  inline void init()
  {
    error = baseError;
  }

  /**
   * Increments the coordinates to the next point on the line.
   * @param pos The position of the current pixel on the line, which is incremented by the
   * Bresenham algorithm
   */
  inline void getNext(Vector2<int>& pos)
  {
    pos += standardOffset;
    error += delta;
    if(error > 0)
    {
      pos += correctionOffset;
      error += resetError;
    }
  }

  /**
   * In conjuction with certain constructors (see above), this value can be used as a
   * termination criterion for the scan. In those cases, getNext can be called
   * numberOfPixels times without leaving the image boundaries.
   */
  int numberOfPixels;

private:

  /** Increase x-values, if true. */
  bool alongX;
  /** The error per step. */
  int delta;
  /** The initial error value. */
  int baseError;
  /** Resets the error to a value less than zero. */
  int resetError;
  /** The standard offset per step. */
  Vector2<int> standardOffset;
  /** The additional offset, if the error is above zero. */
  Vector2<int> correctionOffset;
  /** The current error counter. */
  int error;

  /** Computes the Bresenham parameters. */
  void setup(const Vector2<int>& diff);
};
