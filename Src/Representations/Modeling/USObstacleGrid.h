/**
* @file USObstacleGrid.h
*
* Declaration of class USObstacleGrid
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Pose2D.h"


/**
* @class USObstacleGrid
*
* A class that represents the current state robot's environment as measured by its ultrasound sensors
*/
class USObstacleGrid : public Streamable
{
public:
  /** Function for drawing */
  void draw();

  /**
  * @class Cell
  * A cell within the modeled occupancy grid
  */
  class Cell
  {
  public:
    unsigned lastUpdate;                      /** The point of time of the last change of the state*/
    enum State {FREE = 0};                    /** Minimum and maximum value of state */
    int state;                                /** The state of the cell, i.e. the number of positive measurements within the last few frames*/
    int cluster;                              /** The cluster a cell belongs to */

    /** Constructor */
    Cell(): lastUpdate(0), state(FREE), cluster(0)
    {}
  };

  /** Dimensions of the grid */
  enum {CELL_SIZE   = 60,
        GRID_LENGTH = 45,
        GRID_SIZE   = 45 * 45
       };

  Cell cells[GRID_SIZE];          /**< The grid */
  Pose2D drawingOrigin;           /**< Possible origin for drawing */
  int cellOccupiedThreshold;      /**< Threshold as configured for grid generation */
  int cellMaxOccupancy;           /**< Threshold as configured for grid generation */

  /** Streaming function
  * @param in Object for streaming in the one direction
  * @param out Object for streaming in the other direction
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_REGISTER_FINISH();
  }
};
