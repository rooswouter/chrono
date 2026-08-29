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
// =============================================================================

#ifndef CH_VEHICLE_ABS_H
#define CH_VEHICLE_ABS_H

#include <ostream>

#include "chrono_vehicle/advanced_systems/ChVehicleSystem.h"

namespace chrono {
    namespace vehicle {

        /// @addtogroup vehicle_wheeled
        /// @{


        class CH_VEHICLE_API ChABS : public ChVehicleSystem
        {
        public:
            /// Construct a vehicle system with a default ChSystem.
            ChABS(const std::string& filename) {
                ChVehicleSystem::Create(filename);
            }

            /// Construct a vehicle system with a default ChSystem.
            ChABS(const rapidjson::Document& d)  {
                Create(d);
            }

            /// Destructor.
            virtual ~ChABS() {}

            /// Get the name of the vehicle system template.
            virtual std::string GetTemplateName() const
            {
                return "ABS";
            }

            virtual void Initialize(ChWheeledVehicle* vehicle) override;

            virtual void Synchronize(double time,                       ///< [in] current time
                DriverInputs& driver_inputs  ///< [in] current driver inputs
            ) override;

            virtual void Create(const rapidjson::Document& d) override;
        protected:
            bool Apply(std::shared_ptr<ChBrake> brake, std::shared_ptr<ChWheel> wheel, DriverInputs &driver_inputs);

            double m_lockup_speed = 1e-3;

            std::vector<std::pair<std::shared_ptr<ChBrake>, std::shared_ptr<ChWheel>>> m_brake_wheels;

            double m_enabled_time = -1.0;
            double m_engage_time = 0.0;
        };

        /// @} vehicle_wheeled

    }  // end namespace vehicle
}  // end namespace chrono

#endif
