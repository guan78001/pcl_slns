file(GLOB MAIN_SRCS
  "*.[hH]"
  "*.[cC]"
  "*.[hc]pp"
  "*.[hc]xx"
  "*.cc"
)

SET(SRCS
   ${MAIN_SRCS} 
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

#Combine into one for simplicity
set(ALL_EXTERNAL_LIBS
  flann
  boost
  eigen3
  pcl
  vtk-6.2
)
use_libraries(${ALL_EXTERNAL_LIBS})
add_executable(Demo ${SRCS})
target_link_libraries(Demo LINK_PRIVATE ${PCL_LIB})
set_target_properties(Demo PROPERTIES FOLDER "Demo")

set(TARGET_PATH ${EXECUTABLE_OUTPUT_PATH})
set(BUILD_TYPE $<$<CONFIG:debug>:debug>$<$<CONFIG:release>:release>)
message(STATUS ${TARGET_PATH})
#segmentation
SET(PCL_MODULES "common" "features" "filters" "io" "io_ply" "kdtree" "keypoints" "octree" "registration" "sample_consensus" "surface" "search" "tracking" "visualization")
FOREACH(MODULE ${PCL_MODULES})
	add_custom_command(TARGET Demo POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PCL_BIN_DIR}/pcl_${MODULE}_${BUILD_TYPE}.dll" ${TARGET_PATH}/$<CONFIGURATION>
	)
ENDFOREACH(MODULE)
