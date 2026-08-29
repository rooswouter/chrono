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
// Authors: Wouters
// =============================================================================
//
// Base class for a wheeled vehicle model.
//
// =============================================================================

#include "chrono_vehicle/advanced_systems/ChWheeledVehicleAdvanced.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/advanced_systems/ChUtilsAdvancedSystems.h"

using namespace rapidjson;


namespace chrono {
    namespace vehicle {

        ChWheeledVehicleAdvanced::ChWheeledVehicleAdvanced(const std::string& filename, ChContactMethod contact_method, bool create_powertrain, bool create_tires) :
            WheeledVehicle(filename, contact_method, create_powertrain, create_tires)
        {
            CreateSystems(filename);
        }

        ChWheeledVehicleAdvanced::ChWheeledVehicleAdvanced(ChSystem* system, const std::string& filename, bool create_powertrain, bool create_tires) :
            WheeledVehicle(system, filename, create_powertrain, create_tires)
        {
            CreateSystems(filename);
        }

        void ChWheeledVehicleAdvanced::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel)
        {
            WheeledVehicle::Initialize(chassisPos, chassisFwdVel);
            for (auto it = m_vehicle_systems.begin(); it != m_vehicle_systems.end(); ++it) {
                it->second->Initialize(this);
            }

        }

        void ChWheeledVehicleAdvanced::CreateSystems(const std::string& filename)
        {
            // Open and parse the input file
            Document d;
            ReadFileJSON(filename, d);
            if (d.IsNull())
                return;

            if (!d.HasMember("Vehicle Systems") || !d["Vehicle Systems"].IsArray()) {
                return;
            }

            for (unsigned int i = 0; i < d["Vehicle Systems"].Size(); i++) {
                std::shared_ptr<ChVehicleSystem> vehicleSystem = ReadVehicleSystemJSON(GetVehicleDataFile(d["Vehicle Systems"][i]["Input File"].GetString()));
                if (vehicleSystem && d["Vehicle Systems"][i].HasMember("Name")) {
                    // Overwrite the name if specified in the Vehicle file as well
                    vehicleSystem->SetName(d["Vehicle Systems"][i]["Name"].GetString());
                    m_vehicle_systems.insert(std::make_pair(vehicleSystem->GetName(), vehicleSystem));
                }
            }

        }

        void ChWheeledVehicleAdvanced::Synchronize(double time, const DriverInputs& driver_inputs)
        {
            DriverInputs controlled_driver_inputs = driver_inputs;
            for (auto it = m_vehicle_systems.begin(); it != m_vehicle_systems.end(); ++it) {
                it->second->Synchronize(time, controlled_driver_inputs);
            }
            WheeledVehicle::Synchronize(time, controlled_driver_inputs);
        }


        void ChWheeledVehicleAdvanced::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain)
        {
            WheeledVehicle::Synchronize(time, driver_inputs, terrain);
        }

        bool ChWheeledVehicleAdvanced::AddVehicleSystem(const std::string &name, std::shared_ptr<ChVehicleSystem> vehicle_system)
        {
            // Ensure name is not used yet
            if (m_vehicle_systems.find(name) != m_vehicle_systems.end()) {
                return false;
            }
            m_vehicle_systems.insert(std::make_pair(name, vehicle_system));
            return true;
        }

        bool ChWheeledVehicleAdvanced::RemoveVehicleSystem(const std::string &name)
        {
            // Ensure name is not used yet
            auto it = m_vehicle_systems.find(name);
            if (it == m_vehicle_systems.end()) {
                return false;
            }
            m_vehicle_systems.erase(it);
            return true;
        }

        std::shared_ptr<ChVehicleSystem> ChWheeledVehicleAdvanced::GetVehicleSystem(const std::string& name)
        {
            auto it = m_vehicle_systems.find(name);
            if (it == m_vehicle_systems.end()) {
                return nullptr;
            }
            return it->second;
        }


        // -----------------------------------------------------------------------------

        std::string ChWheeledVehicleAdvanced::ExportComponentList() const
        {
            return WheeledVehicle::ExportComponentList();
        }

        void ChWheeledVehicleAdvanced::WriteOutput(int frame, double time) const
        {
            WheeledVehicle::WriteOutput(frame, time);
        }

        // -----------------------------------------------------------------------------

        void ChWheeledVehicleAdvanced::SaveCheckpoint(ChCheckpoint& database) const
        {
            WheeledVehicle::SaveCheckpoint(database);
        }

        void ChWheeledVehicleAdvanced::LoadCheckpoint(ChCheckpoint& database)
        {
            WheeledVehicle::LoadCheckpoint(database);
        }

    }  // end namespace vehicle
}  // end namespace chrono
