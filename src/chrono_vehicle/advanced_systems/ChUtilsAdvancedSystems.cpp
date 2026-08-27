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

#include <fstream>
#include <unordered_map>
#include <utility>
#include <vector>


#include "chrono_vehicle/advanced_systems/ChUtilsAdvancedSystems.h"

#include "chrono_vehicle/advanced_systems/EngineShaftsAdvanced.h"


namespace chrono {
namespace vehicle {
	void RegisterAdvancedSystems()
	{
		// Engine
        std::function<std::shared_ptr<ChEngine>(const rapidjson::Document& d)>;

		std::function<std::shared_ptr<ChEngine>(const rapidjson::Document& d)> engineFactory = [](const rapidjson::Document& d) {
            return chrono_types::make_shared<EngineShaftsAdvanced>(d);
        };
        RegisterEngineJSONFactory("EngineShaftsAdvanced", engineFactory);
	}
}
}  // namespace chrono