_`Metadata Configuration`
=========================

_`Project-level Settings`
-------------------------

The metadata.json file is the single source of truth for food_volume_measure. It carries the package identity together with every build, dependency and documentation switch, so the CMake configuration, the Conan recipe and the documentation pipeline all read the same values instead of duplicating them.

_`File Structure`
-----------------

The JSON file is organized in sections: the package identity (name, target, version, team, license, description, authors, maintainers, topics, url and homepage), the build configuration (cmake_version, build_cppstd, build_cstd, build_type, activate_code_coverage, is_shared, is_header, generate_modules_inplace, std_modules and user_modules), the dependencies object, baremetal_white_list, graphviz_bin, the documentation settings (doc_languages, doc_versions, doc_doxygen_folders and doc_doxygen_suffix), and the automation switches (trigger_tests, saving_tests_log, enable_python_bindings and workflow_triggers).

_`Key Settings`
---------------

The build configuration fixes the toolchain expectation and the language standards used for the library targets. The dependencies object is split into four buckets - common, c, cpp and infra - and every entry maps a Conan package name to the CMake targets that should be linked into the matching library. baremetal_white_list names the packages that survive a bare-metal cross build. The documentation settings decide which languages and versions are generated and which folders and file suffixes Doxygen scans.

_`Example Configuration`
------------------------

The settings currently used by this library are:

.. code-block:: json

   {
     "name": "food_volume_measure",
     "version": "0.1.0",
     "team": "HeT-FTI",
     "license": "Apache-2.0",
     "build_cppstd": "17",
     "build_cstd": "11",
     "is_shared": false,
     "is_header": false,
     "enable_python_bindings": true,
     "dependencies": {
       "common": {"ZLIB": ["ZLIB::ZLIB"]},
       "c": {"PCRE2": ["pcre2::pcre2"]},
       "cpp": {
         "Eigen3": ["Eigen3::Eigen"],
         "etl": ["etl::etl"],
         "PCL": ["PCL::PCL"],
         "nlohmann_json": ["nlohmann_json::nlohmann_json"]
       },
       "infra": {"GTest": ["gtest::gtest"], "pybind11": ["pybind11::module"]}
     },
     "baremetal_white_list": ["etl", "ArduinoJson"],
     "doc_languages": ["en", "zh"],
     "doc_versions": ["1.0", "2.0"]
   }
