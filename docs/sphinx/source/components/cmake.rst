_`CMakeLists Configuration`
===========================

_`Overview`
-----------

CMakeLists.txt is where the build starts. It reads the package identity and every build switch
from metadata.json, collects the sources from include and src, and creates the library targets
and their installation rules.

_`Key Features`
---------------

- Package identity and build switches read from metadata.json
- Automatic source collection and target creation
- Dependencies wired in through the C_DEPS / CPP_DEPS lists the Conan recipe prepares
- Optional C++ module support (C++23 and above)
- Optional Python bindings module
- Installation rules for libraries, headers and the CMake package export

_`Main Configurations`
----------------------

Build switches come from metadata.json instead of being hard-coded: project name, namespace and
version, the C and C++ standards, shared or static, header-only mode, the pre-compiled standard
modules list, code coverage and the Python bindings switch.

Sources are collected recursively from include (\*.h, \*.hpp) and src (\*.c, \*.cpp and the
module suffixes \*.ixx / \*.cppm). Each glob is anchored to its own directory, so the scan does
not pick up api, test_package, benchmark or build.

For a general package it creates food_volume_measure_c and food_volume_measure_cpp, plus an
INTERFACE target that links both, so a consumer only has to link food_volume_measure. The
language standards are set per target (C11 and C++17 here) with compiler extensions disabled.
add_library() does not accept an empty source list, so a generated empty translation unit keeps
the C target valid even though this library has no C sources. On Windows a requested shared
build falls back to static with a warning.

The Conan recipe passes dependencies in as Pkg@Targets pairs. Each pair is split, find_package()
is called, and the listed targets are linked into the C or the C++ library.

C++ modules are only enabled for C++23 and above; below that the build prints a note saying
module support is off. The module file suffix is platform-specific: MSVC uses .ixx and GCC/Clang
use .cppm. With the Python bindings switch on, an extra pybind11 module is built from
api/python_bindings.cpp and installed next to the libraries.