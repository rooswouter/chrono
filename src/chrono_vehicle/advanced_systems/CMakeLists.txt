
if(CH_ENABLE_MODULE_VEHICLE)
  option(CH_ENABLE_MODULE_VEHICLE_ADVANCED_SYSTEMS "Build the Chrono Vehicle Advanced Systems module" TRUE)
  set(CH_ENABLE_MODULE_VEHICLE_ADVANCED_SYSTEMS "${CH_ENABLE_MODULE_VEHICLE_ADVANCED_SYSTEMS}" PARENT_SCOPE)
else()
  set(CH_ENABLE_MODULE_VEHICLE_ADVANCED_SYSTEMS FALSE CACHE BOOL "Build the Chrono vehicle models library" FORCE)
endif()

if(CH_ENABLE_MODULE_VEHICLE_ADVANCED_SYSTEMS)
  message(STATUS "\n==== Chrono Vehicle Advanced Systems ====\n")
endif()


install(FILES ChApiModels.h DESTINATION include/chrono_vehicle/advanced_systems)

set(CV_VEHICLE_ADVANCED_SYSTEM_FILES
    ChUtilsAdvancedSystems.cpp
    ChUtilsAdvancedSystems.h
    ChEngineShaftsAdvanced.cpp
    ChEngineShaftsAdvanced.h
    EngineShaftsAdvanced.cpp
    EngineShaftsAdvanced.h
    ChShaftsThermalEngineAdvanced.cpp
    ChShaftsThermalEngineAdvanced.h
)

source_group("advanced_systems" FILES ${CV_VEHICLE_ADVANCED_SYSTEM_FILES})

message(STATUS "Add Chrono_vehicle_advanced_systems library")

add_library(Chrono_vehicle_advanced_systems
#
    ${CV_VEHICLE_ADVANCED_SYSTEM_FILES}
)

add_library(Chrono::vehicle_advanced_systems ALIAS Chrono_vehicle_advanced_systems)

set_target_properties(Chrono_vehicle_advanced_systems PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_vehicle_advanced_systems PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_vehicle_advanced_systems PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_vehicle_advanced_systems PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_vehicle_advanced_systems PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_link_libraries(Chrono_vehicle_advanced_systems PUBLIC Chrono_core)
target_link_libraries(Chrono_vehicle_advanced_systems PUBLIC Chrono_vehicle)


target_compile_definitions(Chrono_vehicle_advanced_systems INTERFACE 
    $<BUILD_INTERFACE:CHRONO_VEHICLE_DATA_DIR=\"${PROJECT_BINARY_DIR}/bin/data/vehicle/\">
    $<INSTALL_INTERFACE:CHRONO_VEHICLE_DATA_DIR=\"$<INSTALL_PREFIX>/data/vehicle/\">)

target_compile_definitions(Chrono_vehicle_advanced_systems PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_VEHICLE>)

install(TARGETS Chrono_vehicle_advanced_systems
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_vehicle_advanced_systems)