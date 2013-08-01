/**
* @file XabslSymbolValue.h
*
* Definition of base classes for symbol input and output values.
*
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/

#pragma once

namespace xabsl
{

/**
* @class SymbolInputValue
* The abstract base class for input symbol values.
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/
template<class T> class SymbolInputValue
{
public:
  /** virtual destructor */
  virtual ~SymbolInputValue() {}

  /** returns the value of the parameter */
  virtual T getValue() = 0;
};

/**
* @class SymbolOutputValue
* The abstract base class for output symbol values.
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/
template<class T> class SymbolOutputValue
{
public:
  /** virtual destructor */
  virtual ~SymbolOutputValue() {}

  /** returns the value of the parameter */
  virtual T getValue() = 0;

  /** sets the value of the parameter */
  virtual void setValue(T) = 0;
};

} // namespace
