#pragma once
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

#ifndef SHAFTS_ENGINE_ADVANCED_H
#define SHAFTS_ENGINE_ADVANCED_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/advanced_systems/ChEngineShaftsAdvanced.h"

namespace chrono {
    namespace vehicle {

        /// @addtogroup vehicle_powertrain
        /// @{

        /// Shafts-based Advanced engine subsystem (specified through JSON file).
        class CH_VEHICLE_API EngineShaftsAdvanced : public ChEngineShaftsAdvanced
        {
        public:
            EngineShaftsAdvanced(const std::string& filename);
            EngineShaftsAdvanced(const rapidjson::Document& d);
            ~EngineShaftsAdvanced() {}

            virtual double GetMotorBlockInertia() const override { return m_motorblock_inertia; }
            virtual double GetMotorshaftInertia() const override { return m_motorshaft_inertia; }

            virtual void SetEngineTorqueMap(std::shared_ptr<ChFunctionInterp2D>& map) override;

        private:
            virtual void Create(const rapidjson::Document& d) override;

            double m_motorblock_inertia;
            double m_motorshaft_inertia;

            ChMap2DData m_engine_torque;
        };

        /// @} vehicle_powertrain

    }  // end namespace vehicle
}  // end namespace chrono

#endif
