// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Wouter Roos
// =============================================================================
//
// Utility functions for parsing Advanced Systems.
//
// =============================================================================

#ifndef CH_UTILS_ADVANCED_SYSTEMS_H
#define CH_UTILS_ADVANCED_SYSTEMS_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_thirdparty/rapidjson/document.h"

#include "chrono_vehicle/advanced_systems/ChVehicleSystem.h"

// Factory registration
//using factoryName ## JSONFactory = std::function<std::shared_ptr<objectType>(const rapidjson::Document& d)>; \
//

#if 0
#define CREATE_FACTORY(factoryName, objectType) \
namespace { \
    std::unordered_map<std::string, factoryName ## JSONFactory>& Get ## factoryName ## JSONFactories() { \
        static std::unordered_map<std::string, factoryName ## JSONFactory> factories; \
        return factories; \
    } \
} \
CH_VEHICLE_API void Register ## factoryName ## JSONFactory(const std::string& template_name, factoryName ## JSONFactory factory) { \
    Get ## factoryName ## JSONFactories()[template_name] = std::move(factory); \
} \
CH_VEHICLE_API void Unregister ## factoryName ## SONFactory(const std::string& template_name) { \
    Get ## factoryName ## JSONFactories().erase(template_name); \
}
#endif
namespace chrono {
namespace vehicle {
class ChGearChanger;

/// todo: Create a macro for the factory methods?
/// 
/// Factory function type used to create an engine from a parsed JSON document.
/// The document already corresponds to a valid engine specification file.
using GearChangerJSONFactory = std::function<std::shared_ptr<ChGearChanger>(const rapidjson::Document& d)>;
using VehicleSystemJSONFactory = std::function<std::shared_ptr<ChVehicleSystem>(const rapidjson::Document& d)>;

#if 0
/// Register a factory function for an GearChanger JSON template name.
/// If a factory is already registered for the given template name, it is replaced.
/// Registered factories are consulted by ReadGearChangerJSON before the built-in templates.
CH_VEHICLE_API void RegisterGearChangerJSONFactory(const std::string& template_name, GearChangerJSONFactory factory);
CH_VEHICLE_API void RegisterVehicleSystemJSONFactory(const std::string& template_name, VehicleSystemJSONFactory factory);

/// Unregister a previously registered GearChanger JSON factory (no-op if not registered).
CH_VEHICLE_API void UnregisterGearChangerJSONFactory(const std::string& template_name);
CH_VEHICLE_API void UnregisterVehicleSystemJSONFactory(const std::string& template_name);
#endif

/// Register all factories to the JSON reader
CH_VEHICLE_API void RegisterAdvancedSystems();

CH_VEHICLE_API std::shared_ptr<ChGearChanger> ReadGearChangerJSON(const std::string& filename);

CH_VEHICLE_API std::shared_ptr<ChVehicleSystem> ReadVehicleSystemJSON(const std::string& filename);

}
}  // namespace chrono

#endif