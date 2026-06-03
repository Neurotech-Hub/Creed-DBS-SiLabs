# Define minimal required version of CMake.
cmake_minimum_required(VERSION "3.25")

# Project definition
project(App)

include(ExternalProject)

ExternalProject_Add(Creed_DBS_app
  SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/.././cmake_gcc
  BINARY_DIR ${CMAKE_CURRENT_LIST_DIR}/.././cmake_gcc/build
  CMAKE_ARGS --preset project
  BUILD_COMMAND ${CMAKE_COMMAND} --build . --config $<CONFIG>
  INSTALL_COMMAND ""
  BUILD_ALWAYS true
  USES_TERMINAL_BUILD 1
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/.././cmake_gcc
)


set(post_build_command )


add_custom_target(postbuild ALL
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMAND ${post_build_command}
)

add_dependencies(postbuild Creed_DBS_app)

# BEGIN_SIMPLICITY_STUDIO_METADATA=eJx1kc1OwzAQhF8l8h1vC+ISBSpoOERKGoRbCaRIkXG2rSGxo9htihDvztL/Hjiu9c3MzvqbiSR7TpNxMn0rxXQWJ3kp8nQ2TfJJmeXxU8pCFo02TR2ssXPamruCDfmgYAEaZSttFvTwIMZJUrDRfWGiTNh65Ql0AamMCzdOE7L0vg0B+r7n/Q233QKuB4MhvGapUEts5JU2zkujkJxJERo7kQ26VircAalV0u/yz7ycruW748o24Bwcojkq25ET7RMEkdu/bieaD1Rmq1WNLzjHjrqgC3RF5q5WvLfd5zaaV9LLgsFRGmcxzrXRf/r/+T1OgrazH6h8YKgMweMOsSrjR1HKtqWm9akUzBwdGBZSV1pJELppa620/xJ+VWkL69vyGAMXPsDPNoSLFXcHgLMLRHD6IBrZzy8q5LDr=END_SIMPLICITY_STUDIO_METADATA