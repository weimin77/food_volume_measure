_`CMakeLists Configuration`
===========================

_`Overview`
-----------

The CMakeLists.txt file is the build entry point of food_volume_measure. It reads the package identity and every build switch from metadata.json, collects the sources from include and src, and creates the library targets together with their installation rules.

_`Key Features`
---------------

- Package identity and build switches parsed from metadata.json
- Automatic source collection and target creation
- Dependency wiring through the C_DEPS / CPP_DEPS lists prepared by the Conan recipe
- Optional C++ module support (C++23 and above)
- Optional Python bindings module
- Installation rules for libraries, headers and the CMake package export

_`Main Configurations`
----------------------

Build switches are read from metadata.json rather than being hard-coded: project name, namespace and version, the C and C++ standards, shared or static, header-only mode, the pre-compiled standard modules list, code coverage and the Python bindings switch.

Sources are collected recursively from include (\*.h, \*.hpp) and src (\*.c, \*.cpp and the module suffixes \*.ixx / \*.cppm). Every glob is anchored to its own directory, so the scan does not pick up api, test_package, benchmark or build.

For a general package the configuration creates food_volume_measure_c and food_volume_measure_cpp and an INTERFACE target that links both, so a consumer only has to link food_volume_measure. The standards are applied per target - C11 and C++17 for this library - with compiler extensions disabled. Because add_library() rejects an empty source list, a generated empty translation unit keeps the C target valid even though this library currently has no C sources. On Windows a requested shared build is downgraded to static with a warning.

Dependencies arrive from the Conan recipe as Pkg@Targets pairs; the configuration splits each pair, calls find_package() and links the listed targets into the C or the C++ library.

C++ modules are only activated for C++23 and above, and below that the build reports that module support is disabled. Platform-specific module handling covers MSVC (.ixx) and GCC/Clang (.cppm). When the Python bindings switch is on, an additional pybind11 module is built from api/python_bindings.cpp and installed next to the libraries.
