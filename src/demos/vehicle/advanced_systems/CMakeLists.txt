#=============================================================================
# CMake configuration file for the Advanced Systems demos.
#=============================================================================

#--------------------------------------------------------------
# List all Advanced Systems demos

set(DEMOS
    demo_VEH_Test_AdvancedSystems
    demo_VEH_Test_Advanced_OpenCRGRide
)

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS "Chrono_vehicle")
list(APPEND LIBS "Chrono_vehicle_advanced_systems")
list(APPEND LIBS "ChronoModels_vehicle")

if(CH_ENABLE_MODULE_FSI_SPH)
    list(APPEND LIBS "Chrono_fsisph")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_MUMPS)
    list(APPEND LIBS "Chrono_mumps")
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
    list(APPEND LIBS "Chrono_postprocess")
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_vehicle_vsg")
    if(CH_ENABLE_MODULE_FSI_SPH)
      list(APPEND LIBS "Chrono_fsisph_vsg")
    endif()
endif()

#--------------------------------------------------------------
# Create the executables

set(MODELS ../WheeledVehicleModels.h)

foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp ${MODELS})
    source_group("" FILES ${PROGRAM}.cpp ${MODELS})

    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach()
