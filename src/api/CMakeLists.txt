project(VHACD_API CXX C)
include(${CMAKE_COMMON_INC})

if (NOT NO_OPENCL)
	find_package(OpenCL QUIET)
endif()
if (NOT NO_OPENMP)
	find_package(OpenMP QUIET)
endif()
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_library(vhacdapi SHARED ${PROJECT_CPP_FILES} ${PROJECT_C_FILES} ${PROJECT_INC_FILES} ${PROJECT_INL_FILES})

target_include_directories(vhacdapi PRIVATE ${CMAKE_SOURCE_DIR}/VHACD_Lib/public ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_include_directories(vhacd PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc/> $<INSTALL_INTERFACE:include/>)

target_link_libraries(vhacdapi vhacd)

if (OpenCL_FOUND)
    include_directories("${OpenCL_INCLUDE_DIRS}")
    add_definitions( -DOPENCL_FOUND=1 )
    target_link_libraries(vhacdapi ${OpenCL_LIBRARIES})
endif()

if (NOT WIN32 AND NOT APPLE)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
   target_link_libraries(vhacdapi rt)
endif()

message("[VHACDAPI] \t -> CMAKE_INSTALL_PREFIX " ${CMAKE_INSTALL_PREFIX})
install(TARGETS vhacdapi EXPORT vhacdapi-targets DESTINATION lib)
install(FILES ${PROJECT_INC_FILES} DESTINATION include)
install(FILES ${PROJECT_INL_FILES} DESTINATION include)

