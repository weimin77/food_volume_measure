_`Conanfile Configuration`
==========================

_`Usage`
--------

The conanfile.py is the Conan recipe of food_volume_measure. It declares the requirements, prepares the build files, and defines the package that conan create exports.

_`Main Features`
----------------

- Package metadata and requirement loading
- Dependency selection for regular, bare-metal and test builds
- CMake toolchain generation with dependency wiring
- Optional C++ module generation
- C header compatibility for C consumers
- Binary ABI validation before packaging

_`Core Functionality`
---------------------

The PackageRecipe class initializes the package from metadata.json and takes the requirement list from conandata.yml. gtest is dropped from the runtime requirements because only the tests need it, and pybind11 is required only while enable_python_bindings is on. During a bare-metal cross build only the packages named in baremetal_white_list are kept.

generate() writes the CMake toolchain and the dependency files. The four dependency buckets of metadata.json - common, c, cpp and infra - are merged and serialized into the C_DEPS and CPP_DEPS variables as Pkg@Targets pairs, which is exactly the form CMakeLists.txt expects. A bare-metal build keeps only the whitelisted entries and switches CMAKE_TRY_COMPILE_TARGET_TYPE to STATIC_LIBRARY. The module preprocessing step turns the exported .hpp and .cpp files into C++ module units from their export and attach markers, and C headers are rewritten with extern "C" guards so that C consumers can include them.

build() runs the CMake flow, and _validate_built_archives inspects the produced archives with ar and readelf to detect an ABI mismatch. Because the float ABI and FPU settings arrive as compiler flags rather than as Conan settings, package_id also hashes tools.build:cflags and tools.build:cxxflags; without that, two MCUs sharing an architecture string but not an ABI would reuse the same cached binary and fail at link time.

package() and package_info() then define what is exported and how consumers link against it.
