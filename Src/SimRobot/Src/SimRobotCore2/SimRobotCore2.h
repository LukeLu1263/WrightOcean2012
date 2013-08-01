
#pragma once

#include "../SimRobot/SimRobot.h"

template <typename T> class QList;
class QStringList;

namespace SimRobotCore2
{
  /** The different SimRobotCore objet types */
  enum Kind
  {
    object = 1, /**< An object of the type SimRobotCore2::Object */
    actuator, /**< An object of the type SimRobotCore2::Actuator */
    sensor, /**< An object of the type SimRobotCore2::Sensor */
    simulation, /**< An object of the type SimRobotCore2::Simulation */
  };

  class Object;

  /** 
  * @class Renderer
  * An interface to a renderer that can be used to render objects on an OpenGL context
  */
  class Renderer
  {
  public:

    /** The shading technique used to draw appearances or physical representations */
    enum ShadeMode
    {
      OFF = 0,
      WIREFRAME, 
      FLAT,
      SMOOTH,
    };

    /** A camera control mode */
    enum CameraMode
    {
      TARGETCAM = 0,
      FREECAM,
    };

    /** Specifies the current plane for Drag&Drop translations */
    enum DragAndDropPlane 
    {
      XY_PLANE,
      XZ_PLANE,
      YZ_PLANE
    };

    /** The drag and drop modes for physical objects */
    enum DragAndDropMode
    {
      KEEP_DYNAMICS = 0,
      RESET_DYNAMICS,
      APPLY_DYNAMICS
    };

    /** The kind of drag and drop to initiate */
    enum DragType
    {
      DRAG_NORMAL, /**< drag for moving objects or rotating the camera */
      DRAG_ROTATE, /**< drag for rotating objects */
    };

    /** Flags enable or disable some render features */
    enum RenderFlags
    {
      enableLights           = (1 << 0),
      enableTextures         = (1 << 2),
      enableMultisample      = (1 << 3),
      showPhysics            = (1 << 4),
      showCoordinateSystem   = (1 << 5),
      showSensors            = (1 << 6),
      showControllerDrawings = (1 << 7),
      showAsGlobalView       = (1 << 8),
    };

    /** Virtual destructor */
    virtual ~Renderer() {}

    /** Initializes the currently selected OpenGL context.
    * @parem hasSharedDisplayLists Whether the OpenGL has shared display lists and textures with another context that is already initialized.
    */
    virtual void init(bool hasSharedDisplayLists) = 0;

    /** Draws the scene object on the currently selected OpenGL context. */
    virtual void draw() = 0; 

    /** Sets the size of the currently selected OpenGL renderer device. Call this once at the begining to initialize the size.
    * @param width The width of the renderer device
    * @param height The height of the renderer device
    */
    virtual void resize(float fovy, unsigned int width, unsigned int height) = 0; 

    /**
    * Accesses the size of the OpenGL rendering device that has been set using \c resize before
    * @param width The width of the rendering device
    * @param width The height of the rendering device
    */
    virtual void getSize(unsigned int& width, unsigned int& height) const = 0;

    /**
    * Sets the ShadeMode that is used to render appearances.
    * @param shadeMode The shade mode
    */
    virtual void setSurfaceShadeMode(ShadeMode shadeMode) = 0;

    /**
    * Returns the ShadeMode used to render appearances
    * @return The mode currently used
    */
    virtual ShadeMode getSurfaceShadeMode() const = 0;

    /**
    * Sets the ShadeMode that is used to render physical primitives
    * @return The ShadeMode used to render physics
    */
    virtual void setPhysicsShadeMode(ShadeMode shadeMode) = 0;

    /**
    * Returns the ShadeMode used to render physics
    * @return The mode currently used
    */
    virtual ShadeMode getPhysicsShadeMode() const = 0;

    /**
    * Sets the ShadeMode that is used to render controller 3d drawings
    * @return The ShadeMode used to render physics
    */
    virtual void setDrawingsShadeMode(ShadeMode shadeMode) = 0;

    /**
    * Returns the ShadeMode used to controller 3d drawings
    * @return The mode currently used
    */
    virtual ShadeMode getDrawingsShadeMode() const = 0;

    /**
    * Sets the render flags to enable or disable some render features
    * @param The new render flags
    */
    virtual void setRenderFlags(unsigned int renderFlags) = 0;

    /**
    * Returns the render flags used to enable or disable some render features
    * @return The flags currently set
    */
    virtual unsigned int getRenderFlags() const = 0;

    virtual void zoom(float change) = 0; 

    virtual void setCameraMode(CameraMode mode) = 0;
    virtual CameraMode getCameraMode() const = 0;
    virtual void toggleCameraMode() = 0;
    virtual void resetCamera() = 0;
    virtual void fitCamera() = 0;

    virtual int getFovY() const = 0;

    virtual void setDragPlane(DragAndDropPlane plane) = 0;
    virtual DragAndDropPlane getDragPlane() const = 0;
    virtual void setDragMode(DragAndDropMode mode) = 0;
    virtual DragAndDropMode getDragMode() const = 0;

    virtual bool startDrag(int x, int y, DragType type) = 0;

    /**
    * Accesses the object currently manipulated with a drag & drop operation
    * @return The object
    */
    virtual Object* getDragSelection() = 0;

    virtual bool moveDrag(int x, int y) = 0;
    virtual bool releaseDrag(int x, int y) = 0;
    
    /** Sets the camera moving state (useful for camera navigation with WASD keys)
    * @param left Whether the camera should move to the left
    * @param right Whether the camera should move to the right
    * @param up Whether the camera should move up
    * @param down Whether the camera should move down
    */
    virtual void setCameraMove(bool left, bool right, bool up, bool down) = 0;

    virtual void setCamera(const float* pos, const float* target) = 0;
    virtual void getCamera(float* pos, float* target) = 0;
  };

  /**
  * This is an abstract base class for drawings, which can be implemented
  * inside the controller and executed while drawing the scene.
  */
  class Controller3DDrawing
  {
  public:
    /** Empty virtual destructor */
    virtual ~Controller3DDrawing() {};

    /**
    * Virtual function for drawing commands. Derived classes have to
    * override this function.
    */
    virtual void draw() = 0;
  };

  class Object : public SimRobot::Object
  {
  public:
    /**
    * Returns an object type identifier
    * @return The identifier
    */
    virtual int getKind() const {return object;}

    /** 
    * Returns the position of the object
    * @return The position
    */
    virtual const float* getPosition() const = 0;

    /**
    * Returns the 3x3 rotation matrix that describes the rotation of the object
    * @return The matrix
    */
    virtual const float* getRotation() const = 0;

    /** 
    * Moves the  object to target position.
    * @param object The object to move.
    * @param position The target position.
    */
    virtual void move(const float* position) = 0;

    /**
    * Moves the object to target position and rotation specified as 3x3 rotation matrix.
    * @param object The object to move.
    * @param position The target position.
    * @param rotation The target rotation.
    */
    virtual void move(const float* position, const float* rotation) = 0;

    /**
    * Resets the linear and angular velocity of a body and its child bodies
    */
    virtual void resetDynamics() = 0;

    /** 
    * Registers controller drawings at an object in the simulation scene
    * @param drawing The drawing
    */    
    virtual bool registerDrawing(Controller3DDrawing& drawing) = 0;

    /** 
    * Unregisters controller drawings at an object in the simulation scene
    * @param drawing The drawing
    */
    virtual bool unregisterDrawing(Controller3DDrawing& drawing) = 0;

    /**
    * Creates a new instance of a renderer that can be used for rendering
    * an object within an OpenGL context.
    */
    virtual Renderer* createRenderer() = 0;
  };

  class Sensor : public SimRobot::Object
  {
  public:
    enum SensorType
    {
      boolSensor,
      floatSensor,
      cameraSensor,
      floatArraySensor,
      noSensor
    };

    union Data
    {
      bool boolValue;
      float floatValue;
      const float* floatArray;
      const unsigned char* byteArray;
    };

    virtual int getKind() const {return sensor;}

    virtual SensorType getSensorType() const = 0;
    virtual Data getValue() = 0;
    virtual bool getMinAndMax(float& min, float& max) const = 0;
    virtual const QList<int>& getDimensions() const = 0;
    virtual const QStringList& getDescriptions() const = 0;
    virtual const QString& getUnit() const = 0;

    /**
    * Pre-renders the images of multiple camera sensors of the same type at once which improves the performance of camera image rendering.
    * @param cameras An array of camera sensors
    * @param count The amount of camera sensors in the array
    */
    virtual bool renderCameraImages(Sensor** cameras, unsigned int count) = 0;
  };

  class Actuator : public SimRobot::Object
  {
  public:
    virtual int getKind() const {return actuator;}
    virtual void setValue(float value) = 0;
    virtual bool getMinAndMax(float& min, float& max) const = 0;
  };

  class Simulation2 : public SimRobot::Object
  {
  public:
    virtual int getKind() const {return simulation;}

    /** Returns the length of one simulation step
    * @return The time which is simulated by one step (in s)
    */
    virtual double getStepLength() const = 0;

    /** Returns the current simulation step
    * @return The step
    */
    virtual unsigned int getStep() const = 0;

    /** Returns the current simulation time in seconds, starting with 0.0
    * @return The time (in s)
    */
    virtual double getTime() const = 0;

    /** Returns the current frame rate
    * @return The frame rate in frames per second
    */
    virtual unsigned int getFrameRate() const = 0;
  };
};
