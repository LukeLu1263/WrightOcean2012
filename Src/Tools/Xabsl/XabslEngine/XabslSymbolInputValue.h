/**
* @file XabslSymbolInputValue.h
*
* Definition of classes that represents symbol values.
*
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/

#pragma once

#include "Tools/Xabsl/XabslEngine/XabslSymbolValue.h"

namespace xabsl
{

/**
* @class ISVVariable
* The class represnets a symbol value which consists of one variable in the software environment.
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/
template<class T> class ISVVariable : public SymbolInputValue<T>
{
private:
  const T* variable;

public:
  /**
  * Constructor
  * @param v A pointer to a variable in the software environment
  */
  ISVVariable(const T* v) : variable(v) {};

  /** returns the value of the parameter */
  T getValue()
  {
    return *variable;
  }
};

/**
* @class ISVFunction
* The class represnets a symbol value which consists of a static function in the software environment.
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/
template<class T> class ISVFunction : public SymbolInputValue<T>
{
private:
  T(*function)();

public:
  /**
  * Constructor
  * @param f A pointer to a T returning function in the software environment
  */
  ISVFunction(T(*f)()) : function(f) {};

  /** returns the value of the parameter */
  T getValue()
  {
    return (*function)();
  }
};

/**
* @class ISVMemberFunction
* The class represnets a symbol value which consists of a member function in the software environment.
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/
template<class T, class I> class ISVMemberFunction : public SymbolInputValue<T>
{
private:
  I* object;
  T(I::*function)();

public:
  /**
  * Constructor
  * @param o A pointer to a object that contains the member function
  * @param f A pointer to a T returning member function in the software environment
  */
  ISVMemberFunction(I* o, T(I::*f)()) : object(o), function(f) {};

  /** returns the value of the parameter */
  T getValue()
  {
    return (object->*function)();
  }
};

} // namespace
