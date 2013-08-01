/**
* @file XabslSymbols.h
*
* Definition of class Symbols and helper classes
*
* @author <a href="http://www.martin-loetzsch.de">Martin Loetzsch</a>
* @author <a href="http://www.sim.informatik.tu-darmstadt.de/pers/card/risler.html">Max Risler</a>
* @author <a href="mailto:dhonsel@informatik.uni-bremen.de">Daniel Honsel</a>
*/

#pragma once

#include "XabslTools.h"
#include "XabslParameters.h"
#include "XabslSymbolValue.h"
#include "XabslSymbolInputValue.h"
#include "XabslSymbolOutputValue.h"

namespace xabsl
{

/**
* @class EnumElement
* Represents an enum element that is part of an enumerated input or output symbol.
* @author <a href="http://www.martin-loetzsch.de">Martin Loetzsch</a>
*/
class EnumElement : public NamedItem
{
public:
  /**
  * Constructor
  * @param name The name of the enum element as specified in the XML formalization
  * @param value The value for the element from the software environment
    */
  EnumElement(const char* name, int value)
    : NamedItem(name), v(value) {};

  /** The enum value from a function or variable in the software environment */
  int v;
};
/**
* @class Enumeration
*
* Represents a list of enum elements
*/
class Enumeration : public NamedItem
{
public:
  /**
  * Constructor
  * @param name The name of the enumeration as specified in the XML formalization
  * @param index Index of the enumeration in array enumerations in corresponding engine
  */
  Enumeration(const char* name, int index) : NamedItem(name), index(index) {};

  /** Destructor. Deletes the enum elements */
  ~Enumeration();

  /**
  * Assigns an enum value from a function or variable in the software environment
  * to the enum-element string in the XML formalization.
  */
  NamedArray<EnumElement*> enumElements;

  /** Index of the enumeration in array enumerations in corresponding engine */
  int index;
};

/**
* A Template for the input symbol classes
* @author <a href="http://www.martin-loetzsch.de">Martin Loetzsch</a>
*/
template<class T> class InputSymbol : public NamedItem
{
public:
  /**
  * Constructor
  * @param name The name of the symbol, for debugging purposes
  * @param p A pointer to the parameter that the symbol stands for
  * @param errorHandler A reference to a ErrorHandler instance
  * @param index Index of the symbol in array in corresponding engine
  */
  InputSymbol(const char* name, SymbolInputValue<T>* p, ErrorHandler& errorHandler, int index)
    : NamedItem(name), parameters(errorHandler), pParametersChanged(0), index(index), parameter(p)
  {};

  /** returns the value of the symbol */
  T getValue() const
  {
    return parameter->getValue();
  }

  /** Notify the software environment about a parameter change */
  void parametersChanged() const
  { if(pParametersChanged != 0)(*pParametersChanged)(); }

  /** The parameters of the input symbol*/
  Parameters parameters;

  /** A Pointer to a parameter change notification function in the software environment */
  void (*pParametersChanged)();

  /** Index of the symbol in array in corresponding engine */
  int index;

private:
  /** A pointer to a T returning parameter in the software environment */
  SymbolInputValue<T>* parameter;
};

/**
* @class DecimalInputSymbol
*
* Represents a decimal input symbol of the Engine
*
* @author <a href="http://www.martin-loetzsch.de">Martin Loetzsch</a>
*/
class DecimalInputSymbol : public InputSymbol<float>
{
public:
  /**
  * Constructor
  * @param name The name of the symbol, for debugging purposes
  * @param parameter A pointer to the parameter that the symbol stands for
  * @param errorHandler A reference to a ErrorHandler instance
  * @param index Index of the symbol in array in corresponding engine
  */
  DecimalInputSymbol(const char* name, SymbolInputValue<float>* parameter, ErrorHandler& errorHandler, int index)
    : InputSymbol<float>(name, parameter, errorHandler, index)
  {};
};

/**
* @class BooleanInputSymbol
*
* Represents a boolean input symbol of the Engine
*
* @author <a href="http://www.martin-loetzsch.de">Martin Loetzsch</a>
*/
class BooleanInputSymbol : public InputSymbol<bool>
{
public:
  /**
  * Constructor
  * @param name The name of the symbol, for debugging purposes
  * @param parameter A pointer to the parameter that the symbol stands for
  * @param errorHandler A reference to a ErrorHandler instance
  * @param index Index of the symbol in array in corresponding engine
  */
  BooleanInputSymbol(const char* name, SymbolInputValue<bool>* parameter, ErrorHandler& errorHandler, int index)
    : InputSymbol<bool>(name, parameter, errorHandler, index)
  {};
};

/**
* @class EnumeratedInputSymbol
*
* Represents a enumerated input symbol of the Engine
*
* @author <a href="http://www.martin-loetzsch.de">Martin Loetzsch</a>
*/
class EnumeratedInputSymbol : public InputSymbol<int>
{
public:
  /**
  * Constructor
  * @param name The name of the symbol, for debugging purposes
  * @param enumeration Pointer to the list of enumeration elements
  * @param parameter A pointer to the parameter that the symbol stands for
  * @param errorHandler A reference to a ErrorHandler instance
  * @param index Index of the symbol in array in corresponding engine
  */
  EnumeratedInputSymbol(const char* name, Enumeration* enumeration, SymbolInputValue<int>* parameter, ErrorHandler& errorHandler, int index)
    : InputSymbol<int>(name, parameter, errorHandler, index), enumeration(enumeration)
  {};

  /** Pointer to the list of enumeration elements */
  Enumeration* enumeration;
};

/**
* @class OutputSymbol
*
* A Template for the output symbol classes
*
* @author <a href="http://www.sim.informatik.tu-darmstadt.de/pers/card/risler.html">Max Risler</a>
*/
template<class T> class OutputSymbol : public NamedItem
{
public:
  /**
  * Constructor
  * @param name The name of the symbol, for debugging purposes
  * @param p A pointer to the parameter that the symbol stands for
  * @param index Index of the symbol in array in corresponding engine
  */
  OutputSymbol(const char* name, SymbolOutputValue<T>* p, int index)
    : NamedItem(name), activeValueWasSet(false), index(index), parameter(p)
  {};

  /** Set the value of the symbol. */
  void setValue(T value)
  {
    parameter->setValue(value);
    activeValueWasSet = true;
  }

  /** Returns the current value of the symbol. */
  T getValue() const
  {
    return parameter->getValue();
  }

  /** If true, the value was set during the last execution of the option graph. */
  bool activeValueWasSet;

  /** Index of the symbol in array in corresponding engine */
  int index;

private:
  /** A pointer to the parameter that the symbol stands for */
  SymbolOutputValue<T>* parameter;
};

/**
* @class DecimalOutputSymbol
*
* Represents a decimal output symbol of the Engine
*
* @author <a href="http://www.sim.informatik.tu-darmstadt.de/pers/card/risler.html">Max Risler</a>
*/
class DecimalOutputSymbol : public OutputSymbol<float>
{
public:
  /**
  * Constructor
  * @param name The name of the symbol, for debugging purposes
  * @param parameter A pointer to the variable that the symbol stands for
  * @param index Index of the symbol in array in corresponding engine
  */
  DecimalOutputSymbol(const char* name, SymbolOutputValue<float>* parameter, int index)
    : OutputSymbol<float>(name, parameter, index)
  {};
};

/**
* @class BooleanOutputSymbol
*
* Represents a boolean output symbol of the Engine
*
* @author <a href="http://www.sim.informatik.tu-darmstadt.de/pers/card/risler.html">Max Risler</a>
*
*/
class BooleanOutputSymbol : public OutputSymbol<bool>
{
public:
  /**
  * Constructor
  * @param name The name of the symbol, for debugging purposes
  * @param parameter A pointer to the variable that the symbol stands for
  * @param index Index of the symbol in array in corresponding engine
  */
  BooleanOutputSymbol(const char* name, SymbolOutputValue<bool>* parameter, int index)
    : OutputSymbol<bool>(name, parameter, index)
  {};
};

/**
* @class EnumeratedOutputSymbol
*
* Represents a enumerated output symbol of the Engine
*
* @author <a href="http://www.martin-loetzsch.de">Martin Loetzsch</a>
*/
class EnumeratedOutputSymbol : public OutputSymbol<int>
{
public:
  /**
  * Constructor
  * @param name The name of the symbol, for debugging purposes
  * @param enumeration Pointer to the list of enumeration elements
  * @param parameter A pointer to the variable that the symbol stands for
  * @param index Index of the symbol in array in corresponding engine
  */
  EnumeratedOutputSymbol(const char* name, Enumeration* enumeration, SymbolOutputValue<int>* parameter, int index)
    : OutputSymbol<int>(name, parameter, index), enumeration(enumeration)
  {};

  /** Pointer to the list of enumeration elements */
  Enumeration* enumeration;
};

/**
* @class Symbols
*
* Handles the symbols of the Engine.
*
* @author <a href="http://www.martin-loetzsch.de">Martin Loetzsch</a>
* @author <a href="http://www.sim.informatik.tu-darmstadt.de/pers/card/risler.html">Max Risler</a>
*/
class Symbols
{
public:
  /**
  * Constructor.
  * @param errorHandler Is invoked when errors occur
    */
  Symbols(ErrorHandler& errorHandler)
    : errorHandler(errorHandler) {};

  /** Destructor */
  virtual ~Symbols();

  /**
  * Registers an enum element for an enumeration.
  * @param enumName The name of the enumeration
  * @param name The name of the enum element
  * @param value The value of the element
  */
  void registerEnumElement(const char* enumName,
                           const char* name, int value);

  /**
  * Registers the address of a variable for a decimal input symbol.
  * @param name The name of the symbol
  * @param pVariable A pointer to a variable in the software environment
  */
  void registerDecimalInputSymbol(const char* name, const float* pVariable);

  /**
  * Registers the address of a function for a decimal input symbol.
  * @param name The name of the symbol
  * @param pFunction A pointer to a function that calculates a value for the symbol
  */
  void registerDecimalInputSymbol(const char* name,
                                  float(*pFunction)());

  /**
  * Registers the address of a member function for a decimal input symbol.
  * @param name The name of the symbol
  * @param object A pointer to the object that contains the function
  * @param pFunction A pointer to a member function that calculates a value for the symbol
  */
  template<class T>
  void registerDecimalInputSymbol(const char* name, T* object, float(T::*pFunction)())
  {
    XABSL_DEBUG_INIT(errorHandler.message("registering decimal input symbol \"%s\"", name));

    if(decimalInputSymbols.exists(name))
    {
      errorHandler.error("registerDecimalInputSymbol(): symbol \"%s\" was already registered", name);
      return;
    }
    decimalInputSymbols.append(name, new DecimalInputSymbol(name, new ISVMemberFunction<float, T>(object, pFunction), errorHandler, decimalInputSymbols.getSize()));
  }


  /**
  * Registers the address of a function for parameter change notification for a decimal input symbol.
  * @param name The name of the symbol
  * @param pFunction A pointer to the parameter change notification function
  */
  void registerDecimalInputSymbolParametersChanged(const char* name,
      void (*pFunction)());

  /**
  * Registers a parameter of a parameterized decimal input symbol.
  * @param symbolName The name of the symbol
  * @param name The name of the parameter
  * @param pParam A pointer to the parameter
  */
  void registerDecimalInputSymbolDecimalParameter(const char* symbolName,
      const char* name, float* pParam);
  void registerDecimalInputSymbolBooleanParameter(const char* symbolName,
      const char* name, bool* pParam);
  void registerDecimalInputSymbolEnumeratedParameter(const char* symbolName,
      const char* name, const char* enumName, int* pParam);

  /**
  * Registers the address of a variable for a boolean input symbol.
  * @param name The name of the symbol
  * @param pVariable A pointer to a variable in the software environment
  */
  void registerBooleanInputSymbol(const char* name, const bool* pVariable);

  /**
  * Registers the address of a function for a boolean input symbol.
  * @param name The name of the symbol
  * @param pFunction A pointer to a function that calculates a value for the symbol
  */
  void registerBooleanInputSymbol(const char* name,
                                  bool (*pFunction)());

  /**
  * Registers the address of a member function for a boolean input symbol.
  * @param name The name of the symbol
  * @param object A pointer to the object that contains the function
  * @param pFunction A pointer to a member function that calculates a value for the symbol
  */
  template<class T>
  void registerBooleanInputSymbol(const char* name, T* object, bool (T::*pFunction)())
  {
    XABSL_DEBUG_INIT(errorHandler.message("registering boolean input symbol \"%s\"", name));

    if(booleanInputSymbols.exists(name))
    {
      errorHandler.error("registerBooleanInputSymbol(): symbol \"%s\" was already registered", name);
      return;
    }
    booleanInputSymbols.append(name, new BooleanInputSymbol(name, new ISVMemberFunction<bool, T>(object, pFunction), errorHandler, decimalInputSymbols.getSize()));
  }


  /**
  * Registers the address of a function for parameter change notification for a boolean input symbol.
  * @param name The name of the symbol
  * @param pFunction A pointer to the parameter change notification function
  */
  void registerBooleanInputSymbolParametersChanged(const char* name,
      void (*pFunction)());

  /**
  * Registers a parameter of a parameterized boolean input symbol.
  * @param symbolName The name of the symbol
  * @param name The name of the parameter
  * @param pParam A pointer to the parameter
  */
  void registerBooleanInputSymbolDecimalParameter(const char* symbolName,
      const char* name, float* pParam);
  void registerBooleanInputSymbolBooleanParameter(const char* symbolName,
      const char* name, bool* pParam);
  void registerBooleanInputSymbolEnumeratedParameter(const char* symbolName,
      const char* name, const char* enumName, int* pParam);

  /**
  * Registers the address of a variable for a enumerated input symbol.
  * @param name The name of the symbol
  * @param enumName The name of the associated enumeration
  * @param pVariable A pointer to a variable in the software environment
  */
  void registerEnumeratedInputSymbol(const char* name, const char* enumName, const int* pVariable);

  /**
  * Registers the address of a function for a enumerated input symbol.
  * @param name The name of the symbol
  * @param enumName The name of the associated enumeration
  * @param pFunction A pointer to a function that calculates a value for the symbol
  */
  void registerEnumeratedInputSymbol(const char* name, const char* enumName,
                                     int (*pFunction)());

  /**
  * Registers the address of a member function for a enumerated input symbol.
  * @param name The name of the symbol
  * @param enumName The name of the associated enumeration
  * @param object A pointer to the object that contains the function
  * @param pFunction A pointer to a member function that calculates a value for the symbol
  */
  template<class T>
  void registerEnumeratedInputSymbol(const char* name, const char* enumName, T* object, int (T::*pFunction)())
  {
    XABSL_DEBUG_INIT(errorHandler.message("registering enumerated input symbol \"%s\"", name));

    if(enumeratedInputSymbols.exists(name))
    {
      errorHandler.error("registerEnumeratedInputSymbol(): symbol \"%s\" was already registered", name);
      return;
    }
    if(!enumerations.exists(enumName))
    {
      enumerations.append(enumName, new Enumeration(enumName, enumerations.getSize()));
    }
    enumeratedInputSymbols.append(name, new EnumeratedInputSymbol(name, enumerations[enumName], new ISVMemberFunction<int, T>(object, pFunction), errorHandler, decimalInputSymbols.getSize()));
  }


  /**
  * Registers the address of a function for parameter change notification for an enumerated input symbol.
  * @param name The name of the symbol
  * @param pFunction A pointer to the parameter change notification function
  */
  void registerEnumeratedInputSymbolParametersChanged(const char* name,
      void (*pFunction)());

  /**
  * Registers a parameter of an enumerated input symbol.
  * @param symbolName The name of the symbol
  * @param name The name of the parameter
  * @param pParam A pointer to the parameter
  */
  void registerEnumeratedInputSymbolDecimalParameter(const char* symbolName,
      const char* name, float* pParam);
  void registerEnumeratedInputSymbolBooleanParameter(const char* symbolName,
      const char* name, bool* pParam);
  void registerEnumeratedInputSymbolEnumeratedParameter(const char* symbolName,
      const char* name, const char* enumName, int* pParam);

  /**
  * Registers the address of a variable for a decimal output symbol.
  * @param name The name of the symbol
  * @param pVariable A pointer to a variable in the software environment
  */
  void registerDecimalOutputSymbol(const char* name, float* pVariable);

  /**
  * Registers the address of a function for a decimal output symbol.
  * @param name The name of the symbol
  * @param pSetFunction A pointer to a function that sets a value for the symbol
  * @param pGetFunction A pointer to a function that returns a value for the symbol
  */
  void registerDecimalOutputSymbol(const char* name,
                                   void (*pSetFunction)(float),
                                   float(*pGetFunction)()
                                  );

  /**
  * Registers the address of a function for a decimal output symbol.
  * @param name The name of the symbol
  * @param object A pointer to the object that contains the function
  * @param pSetFunction A pointer to a member function that sets a value for the symbol
  * @param pGetFunction A pointer to a member function that returns a value for the symbol
  */
  template<class T>
  void registerDecimalOutputSymbol(const char* name, T* object, void (T::*pSetFunction)(float), float(T::*pGetFunction)())
  {
    XABSL_DEBUG_INIT(errorHandler.message("registering decimal output symbol \"%s\"", name));

    if(decimalOutputSymbols.exists(name))
    {
      errorHandler.error("registerDcimalOutputSymbol(): symbol \"%s\" was already registered", name);
      return;
    }
    decimalOutputSymbols.append(name, new DecimalOutputSymbol(name, new OSVMemberFunction<float, T>(object, pGetFunction, pSetFunction), decimalOutputSymbols.getSize()));
  }

  /**
  * Registers the address of a variable for a boolean output symbol.
  * @param name The name of the symbol
  * @param pVariable A pointer to a variable in the software environment
  */
  void registerBooleanOutputSymbol(const char* name, bool* pVariable);

  /**
  * Registers the address of a function for a boolean output symbol.
  * @param name The name of the symbol
  * @param pSetFunction A pointer to a function that sets a value for the symbol
  * @param pGetFunction A pointer to a function that returns a value for the symbol
  */
  void registerBooleanOutputSymbol(const char* name,
                                   void (*pSetFunction)(bool),
                                   bool (*pGetFunction)()
                                  );

  /**
  * Registers the address of a function for a boolean output symbol.
  * @param name The name of the symbol
  * @param object A pointer to the object that contains the function
  * @param pSetFunction A pointer to a member function that sets a value for the symbol
  * @param pGetFunction A pointer to a member function that returns a value for the symbol
  */
  template<class T>
  void registerBooleanOutputSymbol(const char* name,  T* object, void (T::*pSetFunction)(bool), bool (T::*pGetFunction)())
  {
    XABSL_DEBUG_INIT(errorHandler.message("registering boolean output symbol \"%s\"", name));

    if(booleanOutputSymbols.exists(name))
    {
      errorHandler.error("registerBooleanOutputSymbol(): symbol \"%s\" was already registered", name);
      return;
    }
    booleanOutputSymbols.append(name, new BooleanOutputSymbol(name, new OSVMemberFunction<bool, T>(object, pGetFunction, pSetFunction), decimalOutputSymbols.getSize()));
  }

  /**
  * Registers the address of a variable for a enumerated output symbol.
  * @param name The name of the symbol
  * @param enumName The name of the associated enumeration
  * @param pVariable A pointer to a variable in the software environment
  */
  void registerEnumeratedOutputSymbol(const char* name, const char* enumName, int* pVariable);

  /**
  * Registers the address of a function for a enumerated output symbol.
  * @param name The name of the symbol
  * @param enumName The name of the associated enumeration
  * @param pSetFunction A pointer to a function that sets a value for the symbol
  * @param pGetFunction A pointer to a function that returns a value for the symbol
  */
  void registerEnumeratedOutputSymbol(const char* name, const char* enumName,
                                      void (*pSetFunction)(int),
                                      int (*pGetFunction)()
                                     );

  /**
  * Registers the address of a function for a enumerated output symbol.
  * @param name The name of the symbol
  * @param enumName The name of the associated enumeration
  * @param object A pointer to the object that contains the function
  * @param pSetFunction A pointer to a member function that sets a value for the symbol
  * @param pGetFunction A pointer to a member function that returns a value for the symbol
  */
  template<class T>
  void registerEnumeratedOutputSymbol(const char* name, const char* enumName, T* object, void (T::*pSetFunction)(int), int (T::*pGetFunction)())
  {
    XABSL_DEBUG_INIT(errorHandler.message("registering enumerated output symbol \"%s\"", name));

    if(enumeratedOutputSymbols.exists(name))
    {
      errorHandler.error("registerEnumeratedOutputSymbol(): symbol \"%s\" was already registered", name);
      return;
    }
    if(!enumerations.exists(enumName))
    {
      enumerations.append(enumName, new Enumeration(enumName, enumerations.getSize()));
    }
    enumeratedOutputSymbols.append(name, new EnumeratedOutputSymbol(name, enumerations[enumName], new OSVMemberFunction<int, T>(object, pGetFunction, pSetFunction), decimalOutputSymbols.getSize()));
  }

  /** Sets all output symbols to unset */
  void resetOutputSymbols();

  /** The enumerations */
  NamedArray<Enumeration*> enumerations;

  /** The decimal input symbols */
  NamedArray<DecimalInputSymbol*> decimalInputSymbols;

  /** The boolean input symbols */
  NamedArray<BooleanInputSymbol*> booleanInputSymbols;

  /** The enumerated input symbols */
  NamedArray<EnumeratedInputSymbol*> enumeratedInputSymbols;

  /** The decimal output symbols */
  NamedArray<DecimalOutputSymbol*> decimalOutputSymbols;

  /** The boolean output symbols */
  NamedArray<BooleanOutputSymbol*> booleanOutputSymbols;

  /** The enumerated output symbols */
  NamedArray<EnumeratedOutputSymbol*> enumeratedOutputSymbols;

private:
  /** Is invoked when errors occur */
  ErrorHandler& errorHandler;
};

} // namespace
