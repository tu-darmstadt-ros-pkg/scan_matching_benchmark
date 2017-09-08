google_binary(scan_matching_benchmark_node
  SRCS
    scan_matching_benchmark_node.cpp
)

install(TARGETS scan_matching_benchmark_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

