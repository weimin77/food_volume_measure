_`Conanfile Configuration`
==========================

_`Usage`
--------

conanfile.py is the Conan recipe of food_volume_measure. It declares the requirements, prepares
the build files, and defines the package that conan create exports.

_`Main Features`
----------------

- Loads the package metadata and the requirement list
- Dependency selection for regular, bare-metal and test builds
- Generates the CMake toolchain and wires in the dependencies
- Optional C++ module generation
- C header compatibility for C consumers
- Checks the built archives for ABI mismatches before packaging

_`Core Functionality`
---------------------

PackageRecipe reads the package metadata from metadata.json and the requirement list from
conandata.yml. gtest is left out of the runtime requirements because only the tests need it, and
pybind11 is pulled in only while enable_python_bindings is on. In a bare-metal cross build, only
the packages listed in baremetal_white_list are kept.

generate() writes the CMake toolchain and the dependency files. The four dependency buckets of
metadata.json (common, c, cpp and infra) are merged and written into the C_DEPS and CPP_DEPS
variables as Pkg@Targets entries, which is the form CMakeLists.txt expects. A bare-metal build
keeps only the whitelisted entries and switches CMAKE_TRY_COMPILE_TARGET_TYPE to STATIC_LIBRARY.
The module preprocessing step reads the export and attach markers and turns the exported .hpp
and .cpp files into C++ module units; C headers get extern "C" guards so C code can include
them.

build() runs the CMake flow, and _validate_built_archives inspects the resulting archives with
ar and readelf to catch an ABI mismatch. The float ABI and FPU settings come in as compiler
flags rather than Conan settings, so package_id also hashes tools.build:cflags and
tools.build:cxxflags. Without that, two MCUs with the same architecture string but different
ABIs would share one cached binary and only fail at link time.

package() and package_info() then say what is exported and how consumers link against it.