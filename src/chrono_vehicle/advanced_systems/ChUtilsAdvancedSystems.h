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

// Factory registration

namespace chrono {
namespace vehicle {

/// Register all factories to the JSON reader
CH_VEHICLE_API void RegisterAdvancedSystems();

}
}  // namespace chrono
