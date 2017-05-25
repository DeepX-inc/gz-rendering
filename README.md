# Ignition Rendering

**Rendering classes and functions for robot applications.**

Ignition Rendering is a component in the ignition framework, a set
of libraries designed to rapidly develop robot applications.

  [http://ignitionrobotics.org](http://ignitionrobotics.org)

## Installation

### Dependencies

####OGRE

Ubuntu:

    # this installs ogre 1.9. Alternatively, 1.8 should also work
    sudo apt-get install libogre-1.9-dev libglew-dev

#### OptiX

Download and install by following instructions on NVIDIA website

CUDA:

http://docs.nvidia.com/cuda

Optix:

https://developer.nvidia.com/optix

### Build

Standard installation can be performed in UNIX systems using the following
steps:

 - mkdir build/
 - cd build/
 - cmake ..
 - sudo make install


### Troubleshooting

if  you encounter errors about different exception specifiers in optix math,
edit `[optix_install_dir]/include/optixu/optixu_math_namespace.h` and comment
out the section that defines `fminf`, fmaxf, and `copysignf`

## Uninstallation

To uninstall the software installed with the previous steps:

 - cd build/
 - sudo make uninstall

## Future Work

The following outlines features and changes that should be implemented in the
immediate future:

 - Implement Optix spotlights in `Material.cu`
 - Fix Gazebo primitive geometry mesh normals
 - Ensure UV all texture coordinates match
 - Replace Gazebo references with Ignition code
 - Add missing primitive and composite visuals
 - Add dynamic lines and billboard geometries
 - Move GL context switching into Ogre camera
 - Update examples to use freeglut
 - Update examples to share glut window code
 - Build scene nodes from SDF elements
 - Move `rendering::SceneManager` to Gazebo codebase
 - Employ `common::MaterialManager`
 - Update SDF to include all material properties
 - Update `common::Material` to include all material properties
 - Add skybox functionality (include skyx?)
 - Add render type mask (e.g. gui-only)