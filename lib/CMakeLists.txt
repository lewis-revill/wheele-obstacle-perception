set(WOP_SOURCES
  Location.cpp
  )

include_directories(
  ${WOP_INCLUDE_DIR}
  )

add_library(WOP STATIC ${WOP_SOURCES})
