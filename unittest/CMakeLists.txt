set(TEST_ALL_SOURCES
  test-all.cpp
  )

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME ON)
set(Boost_USE_DEBUG_RUNTIME OFF)
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(Boost_USE_DEBUG_RUNTIME ON)
endif()

find_package(Boost 1.65.0 COMPONENTS program_options REQUIRED)
find_package(JPEG 8 REQUIRED)

add_executable(test-all ${TEST_ALL_SOURCES})

target_include_directories(test-all PUBLIC
  ${PROJECT_BINARY_DIR}
  ${WOP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${JPEG_INCLUDE_DIRS}
  )

target_link_libraries(test-all PUBLIC
  WOP
  ${Boost_LIBRARIES}
  ${JPEG_LIBRARIES}
  )
