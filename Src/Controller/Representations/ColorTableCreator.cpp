/*
 * ColorTableCreator.cpp
 *
 *  Created on: 06.02.2011
 *      Author: moe
 */

#include "ColorTableCreator.h"
#include "Platform/BHAssert.h"
#include "Color3DTree.h"
#include "Tools/Configuration/ConfigMap.h"
#include <iostream>
#include <algorithm>
#include <list>


ColorClasses::Color ColorTableCreator::eraseColor = ColorClasses::none;

ColorTableCreator::ColorTableCreator()
  : treeRange(20),
    treeNeighbors(3)
{}

ColorTableCreator::~ColorTableCreator() {}

void ColorTableCreator::buildColorTable(ColorTable64& colorTable)
{
  colorTable.clear();
  for(unsigned int i = 0; i < handMadeClassification.size(); ++i)
  {
    ClassifiedColor sample = handMadeClassification[i];
    colorTable.colorClasses[sample.getY() / 4][sample.getU() / 4][sample.getV() / 4] = sample.getColorClass();
  }
}

void ColorTableCreator::buildFinalColorTable(ColorTable64& colorTable)
{
  std::vector<ClassifiedColor> trainingVector;
  for(unsigned int k = 0; k < handMadeClassification.size(); ++k)
  {
    trainingVector.push_back(handMadeClassification[k]);
  }

  colorTable.clear();
  Color3DTree tree(treeRange, treeNeighbors);
  tree.build(trainingVector);
  tree.writeToColorTable(colorTable);
}

void ColorTableCreator::addColorByHand(const Image::Pixel& pixel, const ColorClasses::Color& colorClass)
{
  handMadeClassification.push_back(ClassifiedColor(colorClass, pixel.y, pixel.cb, pixel.cr));
}

bool ColorTableCreator::saveTrainingData(string fileName) const
{
  ConfigMap cm;
  cm["handMadeClassification"] << handMadeClassification;
  ofstream of;
  of.open(fileName.c_str());
  if(!of.is_open())
    return false;
  cm.write(of);
  of.close();
  return true;
}

bool ColorTableCreator::loadTrainingData(string fileName)
{
  handMadeClassification.clear();
  ConfigMap cm;
  if(cm.read(fileName) < 0)
    return false;

  try
  {
    cm["handMadeClassification"] >> handMadeClassification;
  }
  catch(std::invalid_argument e)
  {
    return false;
  } //In this case the Vector was empty

  return true;
}

void ColorTableCreator::removeColor(std::vector<ClassifiedColor>& colors, ColorClasses::Color eColor)
{
  eraseColor = eColor;
  std::vector<ClassifiedColor>::iterator end = colors.end();
  std::vector<ClassifiedColor>::iterator newEnd =
    std::remove_if(colors.begin(), end, ColorTableCreator::checkColor);
  colors.erase(newEnd, end);
}

bool ColorTableCreator::checkColor(ClassifiedColor color)
{
  return color.getColorClass() == eraseColor;
}

void ColorTableCreator::removeHandColor(ColorClasses::Color color)
{
  removeColor(handMadeClassification, color);
}

void ColorTableCreator::replaceColor(ColorClasses::Color search, ColorClasses::Color replaceBy)
{
  for(std::vector<ClassifiedColor>::iterator handIt = handMadeClassification.begin();
      handIt < handMadeClassification.end();
      ++handIt)
  {
    if(handIt->getColorClass() == search)
    {
      (*handIt) = ClassifiedColor(replaceBy, handIt->getY(), handIt->getU(), handIt->getV());
    }
  }
}
