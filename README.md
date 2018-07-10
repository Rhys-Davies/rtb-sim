# rtb-sim

    This was my university thesis project, completed in 2017 under the suprevision of Distinguished Professor
    Peter Corke. It is a class-based interface designed for use with Peter Corke's Robotics Toolbox for MATLAB.
    It wraps V-REP's Remote API MATLAB bindings, and provides a set of classes to mirror simulation objects in the
    MATLAB workspace. The goal is to make it a bit simpler to interact with V-REP in MATLAB.
    
    It is released under an MIT License with no warrenty in the hope anyone who finds it useful expands upon it.

## Operation

    A simulator object, VREP.m, creates and manages the V-REP Remote API connection. 
    'v = VREP();' will by default attempt to connect to the V-REP Remote API on localhost (127.0.0.1:19997).
    VREP.m also wraps the majority of V-REP's Remote API methods, see the documentation in VREP.m for a full list of methods.

    An entity object, sim_entity.m, represents a generic object inside the V-REP scene.
    Factory methods are provided in VREP.m to generate these objects.
    'obj = v.entity('objname');' will create an instance of the sim_entity.m class for the V-REP scene object 'objname'.
    The entity object is subclassed for specific object types such as joints, camera's, LiDAR's, etc.

    The V-REP object types supported in this initial release are:

    *	Generic entities: sim_entity.m
    *	Joints: sim_joint.m
    *	Spherical Joints: sim_spherical_joint.m
    *	Vision Sensors: sim_vision_sensor.m
    *	XY Sensors: sim_xy_sensor.m
    *	XYZ Sensors: sim_xyz_sensor.m
    *	RGB/Greyscale Image Sensors: sim_camera.m
    *	Depth Image Sensor: sim_depth_camera.m
    *	(TRS Task) RGBD Camera: sim_cameraRGBD.m 
    *	Proximity Sensor: sim_proximity_sensor.m
    *	Force Sensor: sim_force_sensor.m
    *	Fast (Image Sensor Type) Hokuyo: sim_fast_hokuyo.m
    *	Arm Assemblies: sim_arm.m
    *	Standard V-REP youBot Model: sim_youBot.m
    *	(TRS Task) Modified youBot Model: sim_youBot_TRS.m
    *	(diffBot Demo) Basic Differential Robot Model in diffBot_Demo: sim_diffBot.m

## Dependencies

    Peter Corke's Robotics Toolbox for MATLAB is required.

## Installation

    Clone, or download as zip and extract, into a folder on your MATLAB path.

    To use the V-REP Remote API the following 3 files must be in a folder 
    on your MATLAB path:

        * remoteApiProto.m
        * remApi.m
        * One of the following library files:
            * remoteApi.dll (Windows)
            * remoteApi.so (Linux)
            * remoteApi.dylib (OSX)
    
    These files are found in:
    "path/to/VREP_install_directory/programming/remoteApiBindings/lib/"
    and
    "path/to/VREP_install_directory/programming/remoteApiBindings/matlab/"
    
    See the documentation in VREP.m for alternatives.

## Examples

    See trs_demo and diffBot_demo repositories.

## License

    This code is provided as-is under an MIT License. Have fun! :)
    
## TODO:
    * Add support for VREP 3.5
    * Seperate the classes representing the diffBot and TRS youBot into their repective repositories.
    * @Folder method of defining VREP.m class. This would make it easier to add factory methods for new object types.
      E.g. @newobjtypefactory.m in @VREP. Would allow factory methods to be provided in packages containing new object types without     
      overwriting/modifying the VREP.m file.
    * Rework Image Sensor handling. There are currently 5 classes representing variations of Image Sensor operations, this seems   
      a bit redundant.
