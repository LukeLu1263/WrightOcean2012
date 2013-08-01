/**
* @file ObstacleModel.cpp
* Implementation of class ObstacleModel
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "ObstacleModel.h"
#include "Tools/Debugging/DebugDrawings.h"


void ObstacleModel::draw()
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModelReduced", "drawingOnField");

  COMPLEX_DRAWING("representation:ObstacleModel",
  {
    for(std::vector<Obstacle>::const_iterator it = obstacles.begin(), end = obstacles.end(); it != end; ++it)
    {
      const Obstacle& obstacle = *it;
      const Vector2<>& a = obstacle.leftCorner;
      const Vector2<>& b = obstacle.rightCorner;
      ColorClasses::Color color = obstacle.type == Obstacle::US ? ColorClasses::robotBlue : obstacle.type == Obstacle::ARM ? ColorClasses::yellow : ColorClasses::black;
      LINE("representation:ObstacleModel", 0, 0, a.x, a.y, 30, Drawings::ps_solid, color);
      LINE("representation:ObstacleModel", 0, 0, b.x, b.y, 30, Drawings::ps_solid, color);
      LINE("representation:ObstacleModel", a.x, a.y, b.x, b.y, 30, Drawings::ps_solid, color);
      CROSS("representation:ObstacleModel", obstacle.center.x, obstacle.center.y, 100, 20, Drawings::ps_solid, ColorClasses::blue);
      CROSS("representation:ObstacleModel", obstacle.closestPoint.x, obstacle.closestPoint.y, 100, 20, Drawings::ps_solid, ColorClasses::red);
    }
  });

  COMPLEX_DRAWING("representation:ObstacleModelReduced",
  {
    for(std::vector<Obstacle>::const_iterator it = obstacles.begin(), end = obstacles.end(); it != end; ++it)
    {
      const Obstacle& obstacle = *it;
      const Vector2<>& a = obstacle.leftCorner;
      const Vector2<>& b = obstacle.rightCorner;
      ColorClasses::Color color = obstacle.type == Obstacle::US ? ColorClasses::robotBlue : obstacle.type == Obstacle::ARM ? ColorClasses::yellow : ColorClasses::black;
      LINE("representation:ObstacleModelReduced", 0, 0, a.x, a.y, 30, Drawings::ps_solid, color);
      LINE("representation:ObstacleModelReduced", 0, 0, b.x, b.y, 30, Drawings::ps_solid, color);
      LINE("representation:ObstacleModelReduced", a.x, a.y, b.x, b.y, 30, Drawings::ps_solid, color);
    }
  });

}
