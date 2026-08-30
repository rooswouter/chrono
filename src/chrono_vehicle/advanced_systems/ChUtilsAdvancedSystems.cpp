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

#include "chrono_vehicle/advanced_systems/ChUtilsAdvancedSystems.h"
#include "chrono_vehicle/advanced_systems/EngineShaftsAdvanced.h"
#include "chrono_vehicle/advanced_systems/ChAutomaticGearChanger.h"
#include "chrono_vehicle/advanced_systems/ChGearChanger.h"
#include "chrono_vehicle/advanced_systems/ChABS.h"
#include "chrono_vehicle/advanced_systems/ChVehicleSystem.h"

#include <unordered_map>

namespace chrono {
    namespace vehicle {

        namespace {

            std::unordered_map<std::type_index, std::unordered_map<std::string, JSONFactoryCreatorErased>>&
                GetJSONFactories()
            {
                static std::unordered_map<std::type_index, std::unordered_map<std::string, JSONFactoryCreatorErased>>
                    factories;
                return factories;
            }

        }  // namespace

        void RegisterJSONFactoryImpl(std::type_index type,
            const std::string& template_name,
            JSONFactoryCreatorErased creator)
        {
            GetJSONFactories()[type][template_name] = std::move(creator);
        }

        void UnregisterJSONFactoryImpl(std::type_index type, const std::string& template_name)
        {
            auto& all = GetJSONFactories();
            auto it = all.find(type);
            if (it == all.end())
                return;
            it->second.erase(template_name);
        }

        JSONFactoryCreatorErased FindJSONFactoryImpl(std::type_index type, const std::string& template_name)
        {
            const auto& all = GetJSONFactories();
            auto type_it = all.find(type);
            if (type_it == all.end())
                return {};
            auto name_it = type_it->second.find(template_name);
            if (name_it == type_it->second.end())
                return {};
            return name_it->second;
        }

        void RegisterAdvancedSystems()
        {
            static bool initialized = false;
            if (initialized)
                return;
            initialized = true;

            RegisterEngineJSONFactory("EngineShaftsAdvanced",
                [](const rapidjson::Document& d) -> std::shared_ptr<ChEngine> {
                    return chrono_types::make_shared<EngineShaftsAdvanced>(d);
                });

            JSONFactory<ChGearChanger>::Register("AutomaticGearChanger",
                [](const rapidjson::Document& d) {
                    return std::make_shared<ChAutomaticGearChanger>(d);
                });

            JSONFactory<ChVehicleSystem>::Register("ABS",
                [](const rapidjson::Document& d) {
                    return std::make_shared<ChABS>(d);
                });
        }

    }  // namespace vehicle
}  // namespace chrono