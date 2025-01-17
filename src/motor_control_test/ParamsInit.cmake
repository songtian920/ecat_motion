

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

function (CreateApp targetName)
    add_executable(${targetName} ${ARGN})
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

    #注册 导出库文件
    install(
           INCLUDES DESTINATION include
    )

endfunction()


