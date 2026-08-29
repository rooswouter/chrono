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

namespace chrono {
    namespace vehicle {
    class ChGearChanger;

    template <typename T> using JSONFactoryCreator = std::function<std::shared_ptr<T>(const rapidjson::Document& d)>;
    template <typename T> class JSONFactory
    {
    public:
        static CH_VEHICLE_API void Register(const std::string& template_name, JSONFactoryCreator<T> creator)
        {
            m_factories[template_name] = std::move(creator);
        }
    
        static CH_VEHICLE_API void Unregister(const std::string& template_name)
        {
            m_factories.erase(template_name);
        }

        static CH_VEHICLE_API const std::unordered_map<std::string, JSONFactoryCreator<T>>& GetCreators()
        {
            return m_factories;
        }

    protected:
        static inline std::unordered_map<std::string, JSONFactoryCreator<T>> m_factories;
    };

    /// Register all factories to the JSON reader
    CH_VEHICLE_API void RegisterAdvancedSystems();

    CH_VEHICLE_API std::shared_ptr<ChGearChanger> ReadGearChangerJSON(const std::string& filename);

    CH_VEHICLE_API std::shared_ptr<ChVehicleSystem> ReadVehicleSystemJSON(const std::string& filename);

    }
}  // namespace chrono

#endif