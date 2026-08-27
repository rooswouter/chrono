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

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_thirdparty/rapidjson/document.h"


// Factory registration

namespace chrono {
namespace vehicle {
class ChGearChanger;

/// todo: Create a macro for the factory methods?
/// 
/// Factory function type used to create an engine from a parsed JSON document.
/// The document already corresponds to a valid engine specification file.
using GearChangerJSONFactory = std::function<std::shared_ptr<ChGearChanger>(const rapidjson::Document& d)>;

/// Register a factory function for an GearChanger JSON template name.
/// If a factory is already registered for the given template name, it is replaced.
/// Registered factories are consulted by ReadGearChangerJSON before the built-in templates.
CH_VEHICLE_API void RegisterGearChangerJSONFactory(const std::string& template_name, GearChangerJSONFactory factory);

/// Unregister a previously registered GearChanger JSON factory (no-op if not registered).
CH_VEHICLE_API void UnregisterGearChangerJSONFactory(const std::string& template_name);

/// Register all factories to the JSON reader
CH_VEHICLE_API void RegisterAdvancedSystems();

CH_VEHICLE_API std::shared_ptr<ChGearChanger> ReadGearChangerJSON(const std::string& filename);

}
}  // namespace chrono
