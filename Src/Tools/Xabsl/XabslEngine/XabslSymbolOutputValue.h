/**
* @file XabslSymbolOutputValue.h
*
* Definition classes for symbol output values.
*
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/

#pragma once

#include "Tools/Xabsl/XabslEngine/XabslSymbolValue.h"

namespace xabsl
{

/**
* @class OSVVariable
* The class represnets a symbol value which consists of one variable in the software environment.
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/
template<class T> class OSVVariable : public SymbolOutputValue<T>
{
private:
  T* variable;

public:
  /**
  * Constructor
  * @param v A pointer to a variable in the software environment
  */
  OSVVariable(T* v) : variable(v) {};

  /** returns the value of the parameter */
  T getValue()
  {
    return *variable;
  }

  /** sets the value of the parameter */
  void setValue(T value)
  {
    *variable = value;
  }

};

/**
* @class OSVFunction
* The class represnets a symbol value which consists of a get and a set function in the software environment.
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/
template<class T> class OSVFunction : public SymbolOutputValue<T>
{
private:
  T(*getFunction)();
  void (*setFunction)(T);

public:
  /**
  * Constructor
  * @param getF A pointer to a function that gets the value of the symbol in the software environment
  * @param setF A pointer to a function that sets the value of the symbol in the software environment
  */
  OSVFunction(T(*getF)(), void (*setF)(T)) : getFunction(getF), setFunction(setF) {};

  /** returns the value of the parameter */
  T getValue()
  {
    return (*getFunction)();
  }

  /** sets the value of the parameter */
  void setValue(T value)
  {
    (*setFunction)(value);
  }
};

/**
* @class OSVMemberFunction
* The class represnets a symbol value which consists of a get and a set member function in the software environment.
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/
template<class T, class I> class OSVMemberFunction : public SymbolOutputValue<T>
{
private:
  I* object;
  T(I::*getFunction)();
  void (I::*setFunction)(T);

public:
  /**
  * Constructor
  * @param o A pointer to a object that contains the member functions
  * @param getF A pointer to a member function that gets the value of the symbol in the software environment
  * @param setF A pointer to a member function that sets the value of the symbol in the software environment
  */
  OSVMemberFunction(I* o, T(I::*getF)(), void (I::*setF)(T)) : object(o), getFunction(getF), setFunction(setF) {};

  /** returns the value of the parameter */
  T getValue()
  {
    return (object->*getFunction)();
  }

  /** sets the value of the parameter */
  void setValue(T value)
  {
    (object->*setFunction)(value);
  }
};

} // namespace
