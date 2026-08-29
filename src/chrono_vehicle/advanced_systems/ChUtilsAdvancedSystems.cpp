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
#include "chrono_vehicle/advanced_systems/ChAutomaticGearChanger.h"
#include "chrono_vehicle/advanced_systems/ChGearChanger.h"
#include "chrono_vehicle/advanced_systems/ChABS.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {
	void RegisterAdvancedSystems()
	{
		// Engine
		std::function<std::shared_ptr<ChEngine>(const rapidjson::Document& d)> engineFactory = [](const rapidjson::Document& d) {
            return chrono_types::make_shared<EngineShaftsAdvanced>(d);
        };
        RegisterEngineJSONFactory("EngineShaftsAdvanced", engineFactory);

        // Gear Changer
        std::function<std::shared_ptr<ChGearChanger>(const rapidjson::Document& d)> gearChangerFactory = [](const rapidjson::Document& d) {
            return chrono_types::make_shared<ChAutomaticGearChanger>(d);
        };
        JSONFactory<ChGearChanger>::Register("AutomaticGearChanger", gearChangerFactory);
        
        // Vehicle Systems
        std::function<std::shared_ptr<ChVehicleSystem>(const rapidjson::Document& d)> absFactory = [](const rapidjson::Document& d) {
            return std::make_shared<ChABS>(d);
        };
        JSONFactory<ChVehicleSystem>::Register("ABS", absFactory);
	}

	std::shared_ptr<ChGearChanger> ReadGearChangerJSON(const std::string& filename) {
        std::shared_ptr<ChGearChanger> gear_changer;

        Document d;
        ReadFileJSON(filename, d);
        if (d.IsNull())
            return nullptr;

        // Check that the given file is a transmission specification file.
        assert(d.HasMember("Type"));
        std::string type = d["Type"].GetString();
        assert(type.compare("GearChanger") == 0);

        // Extract the transmission type.
        assert(d.HasMember("Template"));
        std::string subtype = d["Template"].GetString();

        const auto& factories = JSONFactory<ChGearChanger>::GetCreators();
        auto factory_it = factories.find(subtype);
        if (factory_it != factories.end()) {
            return factory_it->second(d);
        }
        return gear_changer;
    }

    std::shared_ptr<ChVehicleSystem> ReadVehicleSystemJSON(const std::string& filename)
    {
        std::shared_ptr<ChVehicleSystem> vehicle_system;
        Document d;
        ReadFileJSON(filename, d);
        if (d.IsNull())
            return nullptr;

        // Check that the given file is a transmission specification file.
        assert(d.HasMember("Type"));
        std::string type = d["Type"].GetString();
        assert(type.compare("VehicleSystem") == 0);

        // Extract the transmission type.
        assert(d.HasMember("Template"));
        std::string subtype = d["Template"].GetString();

        // Prefer a user-registered factory for this template name.
        const auto& factories = JSONFactory<ChVehicleSystem>::GetCreators();
        auto factory_it = factories.find(subtype);
        if (factory_it != factories.end()) {
            return factory_it->second(d);
        }
        return vehicle_system;
    }

    }
}  // namespace chrono