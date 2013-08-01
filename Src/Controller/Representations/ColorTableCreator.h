/*
 * ColorTableCreator.h
 *
 *  Created on: 06.02.2011
 *      Author: moe
 */

#pragma once

#include "Representations/Configuration/ColorTable64.h"
#include "Tools/Math/Vector3.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Controller/Representations/ClassifiedColor.h"

class ColorTableCreator
{
public:

  typedef std::vector<Image::Pixel > Segment;

  ColorTableCreator();
  virtual ~ColorTableCreator();

  void buildColorTable(ColorTable64& colorTable);
  void buildFinalColorTable(ColorTable64& colorTable);
  void addColorByHand(const Image::Pixel& pixel, const ColorClasses::Color& colorClass);
  void undoLastHandData(ColorTable64& colorTable)
  {
    if(!handMadeClassification.empty())
      handMadeClassification.pop_back();
    buildColorTable(colorTable);
  }
  void resetHandData()
  {
    handMadeClassification.clear();
  }
  bool saveTrainingData(string fileName) const;
  bool loadTrainingData(string fileName);
  bool empty() const { return handMadeClassification.empty();  }
  void removeHandColor(ColorClasses::Color color);
  void removeColor(std::vector<ClassifiedColor>& colors, ColorClasses::Color);
  void replaceColor(ColorClasses::Color search, ColorClasses::Color replaceBy);
  static bool checkColor(ClassifiedColor color);

  int treeRange,
      treeNeighbors;
private:
  static ColorClasses::Color eraseColor;

  std::vector<ClassifiedColor> handMadeClassification;
};
