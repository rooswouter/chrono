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

#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <unordered_map>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono/input_output/ChUtilsJSON.h"
#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
    namespace vehicle {

        template <typename T>
        using JSONFactoryCreator = std::function<std::shared_ptr<T>(const rapidjson::Document& d)>;

        // Type-erased creator stored in the DLL.
        using JSONFactoryCreatorErased =
            std::function<std::shared_ptr<void>(const rapidjson::Document& d)>;

        CH_VEHICLE_API void RegisterJSONFactoryImpl(std::type_index type,
            const std::string& template_name,
            JSONFactoryCreatorErased creator);

        CH_VEHICLE_API void UnregisterJSONFactoryImpl(std::type_index type,
            const std::string& template_name);

        CH_VEHICLE_API JSONFactoryCreatorErased FindJSONFactoryImpl(std::type_index type,
            const std::string& template_name);

        CH_VEHICLE_API void RegisterAdvancedSystems();

        template <typename T>
        class JSONFactory
        {
        public:
            static void Register(const std::string& template_name, JSONFactoryCreator<T> creator)
            {
                RegisterJSONFactoryImpl(typeid(T), template_name,
                    [creator = std::move(creator)](const rapidjson::Document& d) -> std::shared_ptr<void> {
                        return creator(d);  // shared_ptr<T> converts to shared_ptr<void>
                    });
            }

            static void Unregister(const std::string& template_name)
            {
                UnregisterJSONFactoryImpl(typeid(T), template_name);
            }

            static std::shared_ptr<T> ReadJSON(const std::string& filename)
            {
                RegisterAdvancedSystems();

                rapidjson::Document d;
                ReadFileJSON(filename, d);
                if (d.IsNull())
                    return nullptr;

                assert(d.HasMember("Template"));
                const std::string subtype = d["Template"].GetString();

                auto erased = FindJSONFactoryImpl(typeid(T), subtype);
                if (!erased)
                    return nullptr;

                return std::static_pointer_cast<T>(erased(d));
            }
        };

    }  // namespace vehicle
}  // namespace chrono

#endif