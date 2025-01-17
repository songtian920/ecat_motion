cmake_minimum_required(VERSION 3.8)
set(CMAKE_BUILD_TYPE debug)   
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)  #生成compile_commands.json文件

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(ecat_motion LANGUAGES C CXX)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

#
if (NOT DEFINED OUTPUT_PATH)
    get_filename_component(OUTPUT_PATH .. ABSOLUTE)
endif()

set(SDK_SYSTEM System64)

# if (UNIX)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
set(SHARED_LIBS ${OUTPUT_PATH}/sdk/shared/${SDK_SYSTEM}/libpci_zecm.so)
set(SHARED_DLL ${OUTPUT_PATH}/sdk/shared/${SDK_SYSTEM}/libpci_zecm.so)


include_directories(${OUTPUT_PATH}/sdk/include)

if (DEFINED OPEN_WARN)
	if (OPEN_WARN)
        set(WARN_FLAG -Wall)
    else()
        set(WARN_FLAG -w)
    endif()
else()
    set(WARN_FLAG -Wall)
endif()



# include(./ParamsInit.cmake)

#"sync_control1601_test"
set(targetName   "node_motor_controller")


#生成执行档
add_executable(node_motor_controller 
        src/node_motor_controller.cpp src/zy_motor_controller.cpp src/motor_controller.cpp
        include/zy_motor_controller.h include/motor_controller.h)

target_include_directories(node_motor_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
#add_executable(${targetName} ${ARGN})

ament_target_dependencies(
  node_motor_controller
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)

target_compile_options(${targetName} PUBLIC ${WARN_FLAG})
file(GLOB test ${SHARED_LIBS})
if (NOT test)
    add_dependencies(${targetName} pci_zecm_shared)
endif()
target_link_libraries(${targetName} ${SHARED_LIBS})
if (UNIX)
    target_link_libraries(${targetName} -lrt)
endif()


#将执行档复制放到目录lib
install(TARGETS ${targetName}
DESTINATION lib/${PROJECT_NAME})

# 复制ENI文件到指定目录 install/lib
install(FILES ./hcfa_D3E_1602_1A02_ENI.xml DESTINATION lib/${PROJECT_NAME})
# 复制ENI文件到指定目录 build/ecat_motion   CMAKE_BINARY_DIR
file(COPY ./hcfa_D3E_1602_1A02_ENI.xml DESTINATION "${CMAKE_BINARY_DIR}")
#复制第三方库到install/lib
install(FILES ${SHARED_LIBS} DESTINATION lib)


# foreach(app ${Target})
#     CreateApp(${app} ${app}.c)
#     # CreateAppStatic(${app}_static ${app}.c)
#     # install(TARGETS ${app} 
#     #     RUNTIME DESTINATION ${CMAKE_TARGET_OUTPUT_PATH}
#     # )
# endforeach()

ament_package()
