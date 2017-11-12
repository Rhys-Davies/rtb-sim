# rtb-sim

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
