/**
* @file USObstacleGrid.cpp
*
* Implementation of class USObstacleGrid
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "USObstacleGrid.h"
#include "Tools/Debugging/DebugDrawings.h"


void USObstacleGrid::draw()
{
  DECLARE_DEBUG_DRAWING("representation:USObstacleGrid", "drawingOnField");
  DECLARE_DEBUG_DRAWING("origin:USObstacleGrid", "drawingOnField");
  ORIGIN("origin:USObstacleGrid", drawingOrigin.translation.x,
         drawingOrigin.translation.y, drawingOrigin.rotation);
  COMPLEX_DRAWING("representation:USObstacleGrid",
  {
    unsigned char colorOccupiedStep(255 / cellMaxOccupancy);
    ColorRGBA baseColor(200, 200, 255, 128);
    unsigned char cellsForDrawing[GRID_SIZE];
    for(int i = 0; i < GRID_SIZE; ++i)
      cellsForDrawing[i] = colorOccupiedStep* cells[i].state;
    GRID_MONO("representation:USObstacleGrid", CELL_SIZE, GRID_LENGTH, GRID_LENGTH, baseColor, cellsForDrawing);
    const int gridWidth(GRID_LENGTH* CELL_SIZE);
    const int gridHeight(GRID_LENGTH* CELL_SIZE);
    RECTANGLE("representation:USObstacleGrid", -gridWidth / 2, -gridHeight / 2, gridWidth / 2, gridHeight / 2,
              20, Drawings::ps_solid, ColorRGBA(0, 0, 100));
  });
}

