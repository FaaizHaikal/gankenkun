cmake_minimum_required(VERSION 3.5)
project(gankenkun)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gankenkun_interfaces REQUIRED)
find_package(jitsuyo REQUIRED)
find_package(kansei REQUIRED)
find_package(kansei_interfaces REQUIRED)
find_package(keisan REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(tachimawari REQUIRED)
find_package(tachimawari_interfaces REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "version:${OpenCV_VERSION}")
message(STATUS "libraries:${OpenCV_LIBS}")
message(STATUS "OpenCV include path:${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenVINO include path env:${OpenVINO_DIR}/../include")

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/config/node/config_node.cpp"
  "src/${PROJECT_NAME}/node/gankenkun_node.cpp"
  "src/${PROJECT_NAME}/lipm/lipm.cpp"
  "src/${PROJECT_NAME}/walking/node/walking_manager.cpp"
  "src/${PROJECT_NAME}/walking/node/walking_node.cpp"
  "src/${PROJECT_NAME}/walking/kinematics/kinematics.cpp"
  "src/${PROJECT_NAME}/walking/planner/foot_step_planner.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenVINO_DIR}/../include)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  gankenkun_interfaces
  jitsuyo
  kansei
  kansei_interfaces
  keisan
  OpenVINO
  tachimawari
  tachimawari_interfaces
)

target_link_libraries(${PROJECT_NAME}
  openvino::runtime
)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(main "src/gankenkun_main.cpp")
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(main ${PROJECT_NAME})

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_index_cpp
  gankenkun_interfaces
  jitsuyo
  kansei
  kansei_interfaces
  keisan
  rclcpp
  tachimawari
  tachimawari_interfaces)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
