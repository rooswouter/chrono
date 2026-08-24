// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Advanced engine model based on the ChEngineShafts model.
//  - Idle speed
//  - Throttle response
//
// =============================================================================

#ifndef CH_SHAFTS_ADVANCED_ENGINE_H
#define CH_SHAFTS_ADVANCED_ENGINE_H

#include "chrono_vehicle/powertrain/ChEngineShafts.h"


namespace chrono {
    namespace vehicle {

        /// @addtogroup vehicle_powertrain
        /// @{

        /// Advanced engine model based on the template ChEngineShafts class
        class CH_VEHICLE_API ChEngineShaftsAdvanced : public ChEngineShafts
        {
        public:
            virtual ~ChEngineShaftsAdvanced();

            /// Get the name of the vehicle subsystem template.
            virtual std::string GetTemplateName() const override { return "EngineShaftsAdvanced"; }

        protected:
            /// Construct a shafts-based engine model.
            ChEngineShaftsAdvanced(const std::string& name, const ChVector3d& dir_motor_block = ChVector3d(1, 0, 0));

            /// Set Idle Speed [rad/s]
            void SetIdleSpeed(double idle_speed) { m_idle_speed = idle_speed; }
            void SetMaxSpeed(double max_speed) { m_max_speed = max_speed; }

        private:
            /// Initialize this engine system.
            virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

            /// Update the state of this engine system at the current time.
            /// Set the motorshaft speed to the provided value and update the throttle level for the engine.
            virtual void Synchronize(double time,                        ///< current time
                const DriverInputs& driver_inputs,  ///< current driver inputs
                double motorshaft_speed             ///< input transmission speed
            ) override;

            double m_idle_speed = 0.0;  ///< The idle speed of the engine [rad/s]
            double m_max_speed = 0.0;  ///< The idle speed of the engine [rad/s]
        };

        /// @} vehicle_powertrain

    }  // end namespace vehicle
}  // end namespace chrono

#endif
#pragma once
