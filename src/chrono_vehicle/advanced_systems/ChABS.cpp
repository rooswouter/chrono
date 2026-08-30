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
// ABS Vehicle System
//
// =============================================================================

#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/advanced_systems/ChABS.h"


using namespace rapidjson;


namespace chrono {
    namespace vehicle {

        
        void ChABS::Initialize(ChWheeledVehicle* vehicle)
        {
            ChVehicleSystem::Initialize(vehicle);
            //std::vector<std::shared_ptr<ChAxle> >
            // Store all brake / wheel pairs
            for (auto it = vehicle->GetAxles().begin(); it != vehicle->GetAxles().end(); ++it) {
                m_brake_wheels.push_back(std::make_pair((*it)->GetBrake(VehicleSide::LEFT), (*it)->GetWheel(VehicleSide::LEFT)));
                m_brake_wheels.push_back(std::make_pair((*it)->GetBrake(VehicleSide::RIGHT), (*it)->GetWheel(VehicleSide::RIGHT)));
            }
        }

        void ChABS::Synchronize(double time, DriverInputs& driver_inputs)
        {
            if (m_enabled_time > 0.0 && time - m_enabled_time < m_engage_time) {
                driver_inputs.m_braking = 0.0;
                return;
            }
            for (auto it = m_brake_wheels.begin(); it != m_brake_wheels.end(); ++it) {
                if (Apply((*it).first, (*it).second, driver_inputs)) {
                    m_enabled_time = time;
                    return;
                }
            }
        }

        void ChABS::Create(const rapidjson::Document& d)
        {
            // Read top-level data
            assert(d.HasMember("Type"));
            assert(d.HasMember("Template"));
            assert(d.HasMember("Name"));

            std::string name = d["Name"].GetString();
            std::string type = d["Type"].GetString();
            std::string subtype = d["Template"].GetString();

            assert(type.compare("VehicleSystem") == 0);
            assert(subtype.compare("ABS") == 0);

            SetName(name);

            assert(d.HasMember("Lockup Speed"));
            m_lockup_speed = d["Lockup Speed"].GetDouble();
            assert(d.HasMember("Engage Time"));
            m_engage_time = d["Engage Time"].GetDouble();
        }

        bool ChABS::Apply(std::shared_ptr<ChBrake> brake, std::shared_ptr<ChWheel> wheel, DriverInputs& driver_inputs)
        {
            double v = fabs(wheel->GetSpindle()->GetLinVel()[0]);
            if (driver_inputs.m_braking <= 0.0 || v < 0.5) {
                return false;
            }

            //printf("Lin V = %f, %f, %f\n", wheel->GetSpindle()->GetLinVel()[0], wheel->GetSpindle()->GetLinVel()[1], wheel->GetSpindle()->GetLinVel()[2]);

            double slip_ratio = fabs(wheel->GetSpindle()->GetAngVelLocal()[1]) / v;
            if (slip_ratio < m_lockup_speed) {
                driver_inputs.m_braking = 0.0;
                return true;
            }

            return false;
        }

    }  // end namespace vehicle
}  // end namespace chrono
