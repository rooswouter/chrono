// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// ChShaft-based engine model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/core/ChDataPath.h"

#include "chrono_vehicle/advanced_systems/EngineShaftsAdvanced.h"

using namespace rapidjson;

namespace chrono {
    namespace vehicle {

        EngineShaftsAdvanced::EngineShaftsAdvanced(const std::string& filename) : ChEngineShaftsAdvanced("")
        {
            Document d;
            ReadFileJSON(filename, d);
            if (d.IsNull())
                return;

            Create(d);

            std::cout << "Loaded JSON " << filename << std::endl;
        }

        EngineShaftsAdvanced::EngineShaftsAdvanced(const rapidjson::Document& d) : ChEngineShaftsAdvanced("")
        {
            Create(d);
        }

        void EngineShaftsAdvanced::Create(const rapidjson::Document& d)
        {
            // Invoke base class method
            ChPart::Create(d);

            // Read engine data
            m_motorblock_inertia = d["Motor Block Inertia"].GetDouble();
            m_motorshaft_inertia = d["Motorshaft Inertia"].GetDouble();
            
            // Check if we have a full map, or also a throttle scaling

            m_engine_torque.Read(d["Torque Map"]);

            SetIdleSpeed(CH_RPM_TO_RAD_S * d["Idle Speed"].GetDouble());
            SetMaxSpeed(CH_RPM_TO_RAD_S * d["Max Speed"].GetDouble());
        }

        void EngineShaftsAdvanced::SetEngineTorqueMap(std::shared_ptr<ChFunctionInterp2D>& map)
        {
            m_engine_torque.Set(*map, 1.0, CH_RPM_TO_RAD_S, 1.0);
        }

    }  // end namespace vehicle
}  // end namespace chrono
