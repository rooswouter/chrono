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

using namespace rapidjson;

namespace chrono {
namespace vehicle {

    namespace {
            std::unordered_map<std::string, GearChangerJSONFactory>& GetGearChangerJSONFactories() {
            static std::unordered_map<std::string, GearChangerJSONFactory> factories;
            return factories;
        }

    }  // namespace

    // -----------------------------------------------------------------------------

    void RegisterGearChangerJSONFactory(const std::string& template_name, GearChangerJSONFactory factory) {
        GetGearChangerJSONFactories()[template_name] = std::move(factory);
    }

    void UnregisterGearChangerSONFactory(const std::string& template_name) {
        GetGearChangerJSONFactories().erase(template_name);
    }



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
        RegisterGearChangerJSONFactory("AutomaticGearChanger", gearChangerFactory);
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

        // Prefer a user-registered factory for this template name.
        const auto& factories = GetGearChangerJSONFactories();
        auto factory_it = factories.find(subtype);
        if (factory_it != factories.end()) {
            return factory_it->second(d);
        }

        return gear_changer;
    }

    }
}  // namespace chrono