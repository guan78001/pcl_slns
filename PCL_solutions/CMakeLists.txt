cmake_minimum_required(VERSION 2.8)

if(CMAKE_VERSION VERSION_GREATER "2.8.11")
  cmake_policy(SET CMP0022 OLD)
  cmake_policy(SET CMP0040 OLD)
endif()

project(PCL_Solutions)

set(EXECUTABLE_OUTPUT_PATH  "${PROJECT_BINARY_DIR}/bin")
set(LIBRARY_OUTPUT_PATH     "${PROJECT_BINARY_DIR}/bin")

if(CMAKE_GENERATOR MATCHES "Win64")
  set(PLATFORM win64)
else()
  set(PLATFORM win32)
endif()
  
set(3RDPARTY_DIR 	  ${PROJECT_SOURCE_DIR}/3rdParty)

set(PCL_INC)
set(PCL_LIB               debug pcl_common_debug optimized pcl_common_release
                          debug pcl_features_debug optimized pcl_features_release
                          debug pcl_filters_debug optimized pcl_filters_release
                          debug pcl_io_debug optimized pcl_io_release
                          debug pcl_kdtree_debug optimized pcl_kdtree_release
                          debug pcl_keypoints_debug optimized pcl_keypoints_release
                          debug pcl_octree_debug optimized pcl_octree_release
                          debug pcl_sample_consensus_debug optimized pcl_sample_consensus_release
                          debug pcl_search_debug optimized pcl_search_release
                          debug pcl_surface_debug optimized pcl_surface_release
                          debug pcl_visualization_debug optimized pcl_visualization_release)
set(PCL_BIN_DIR           ${3RDPARTY_DIR}/PCL/bin/${PLATFORM})
message(STATUS ${PCL_LIB})
set(VTK_VERSION           6.2)
set(VTK_LIB           vtkChartsCore-${VTK_VERSION}
                          vtkCommonCore-${VTK_VERSION}
			  vtkCommonColor-${VTK_VERSION}
			  vtkCommonMath-${VTK_VERSION}
			  vtkCommonDataModel-${VTK_VERSION}
			  vtkCommonMisc-${VTK_VERSION}
			  vtkCommonSystem-${VTK_VERSION}
			  vtkCommonTransforms-${VTK_VERSION}
			  vtkCommonComputationalGeometry-${VTK_VERSION}
			  vtkCommonExecutionModel-${VTK_VERSION}
			  vtkDICOMParser-${VTK_VERSION}
			  vtksys-${VTK_VERSION}
			  vtkFiltersSources-${VTK_VERSION}
			  vtkFiltersGeneral-${VTK_VERSION}
			  vtkFiltersCore-${VTK_VERSION}
			  vtkFiltersGeometry-${VTK_VERSION}
			  vtkFiltersExtraction-${VTK_VERSION}
			  vtkFiltersStatistics-${VTK_VERSION}
			  vtkFiltersModeling-${VTK_VERSION}
			  vtkFiltersHybrid-${VTK_VERSION}
			  vtkImagingFourier-${VTK_VERSION}
			  vtkImagingCore-${VTK_VERSION}
			  vtkImagingHybrid-${VTK_VERSION}
			  vtkImagingSources-${VTK_VERSION}
			  vtkImagingGeneral-${VTK_VERSION}
			  vtkImagingColor-${VTK_VERSION}
			  vtkInteractionStyle-${VTK_VERSION}
			  vtkalglib-${VTK_VERSION}
			  vtkInfovisCore-${VTK_VERSION}
			  vtkInteractionWidgets-${VTK_VERSION}
			  vtkRenderingCore-${VTK_VERSION}
			  vtkRenderingLOD-${VTK_VERSION}
			  vtkRenderingContextOpenGL-${VTK_VERSION}
			  vtkRenderingContext2D-${VTK_VERSION}
			  vtkRenderingFreeType-${VTK_VERSION}
			  vtkRenderingOpenGL-${VTK_VERSION}
			  vtkRenderingFreeTypeOpenGL-${VTK_VERSION}
			  vtkRenderingVolumeOpenGL-${VTK_VERSION}
			  vtkRenderingVolume-${VTK_VERSION}
			  vtkRenderingAnnotation-${VTK_VERSION}
			  vtkIOPLY-${VTK_VERSION}
			  vtkIOCore-${VTK_VERSION}
			  vtkIOGeometry-${VTK_VERSION}
			  vtkIOLegacy-${VTK_VERSION}
			  vtkIOImage-${VTK_VERSION}
			  vtkzlib-${VTK_VERSION}
			  vtkjsoncpp-${VTK_VERSION}
			  vtkpng-${VTK_VERSION}
			  vtktiff-${VTK_VERSION}
			  vtkjpeg-${VTK_VERSION}
			  vtkmetaio-${VTK_VERSION}
			  vtkftgl-${VTK_VERSION}
			  vtkfreetype-${VTK_VERSION}
			  vtkViewsContext2D-${VTK_VERSION}
			  vtkViewsCore-${VTK_VERSION})
set(VTK_BIN_DIR           ${3RDPARTY_DIR}/vtk-${VTK_VERSION}/bin/${PLATFORM})

macro(use_libraries)
  set(_INC_DIR "")
  set(_LIB_DIR "")
  foreach(LIB ${ARGN})
    #if(EXISTS ${3RDPARTY_DIR}/${LIB}/include)
      set(_INC_DIR ${_INC_DIR} " " ${3RDPARTY_DIR}/${LIB}/include)
    #endif()
    #if(EXISTS ${3RDPARTY_DIR}/${LIB}/lib/${PLATFORM})
      set(_LIB_DIR ${_LIB_DIR} ${3RDPARTY_DIR}/${LIB}/lib/${PLATFORM})
    #endif()
  endforeach()
  include_directories(${_INC_DIR})
  link_directories(${_LIB_DIR})
  
  message(STATUS ${_INC_DIR})
  message(STATUS ${_LIB_DIR})
  
endmacro()

function(PCL_Console proj_name)
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
add_executable(${proj_name} ${SRCS})
target_link_libraries(${proj_name} LINK_PRIVATE ${PCL_LIB} ${VTK_LIB})
set_target_properties(${proj_name} PROPERTIES FOLDER "Demo")

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

endfunction()


add_subdirectory(Demo)
add_subdirectory(OctreeSearch)
add_subdirectory(RangeImageCreation)
add_subdirectory(WritePCD)
add_subdirectory(CloudViewer)
add_subdirectory(Visualizer_Demo)
add_subdirectory(recon_poisson)
add_subdirectory(recon_greedyProjection)
add_subdirectory(recon_marchingCubes)
add_subdirectory(VoxelGridFilter)
add_subdirectory(StatisticalRemoval)
add_subdirectory(RadiusOutlierRemoval)
add_subdirectory(BilateralFilter)