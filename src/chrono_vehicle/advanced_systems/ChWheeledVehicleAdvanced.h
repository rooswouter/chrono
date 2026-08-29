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
// Advanced class for a wheeled vehicle system.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
// Advanced class implements:
// ChVehicleSystem that have access to the vehicle, such as ABS
// todo: advanced loaders
//
// =============================================================================

#ifndef CH_WHEELED_VEHICLE_ADVANCED_H
#define CH_WHEELED_VEHICLE_ADVANCED_H

#include <ostream>

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

namespace chrono {
    namespace vehicle {

        class ChVehicleSystem;

        /// @addtogroup vehicle_wheeled
        /// @{

        /// Base class for chrono wheeled vehicle systems.
        /// This class provides the interface between the vehicle system and other
        /// systems (tires, driver, etc.).
        /// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
        /// pointing forward, and Y-axis towards the left of the vehicle.
        class CH_VEHICLE_API ChWheeledVehicleAdvanced : public WheeledVehicle
        {
        public:
            /// Destructor.
            virtual ~ChWheeledVehicleAdvanced() {}

            /// Get the name of the vehicle system template.
            virtual std::string GetTemplateName() const override { return "WheeledVehicleAdvanced"; }

            virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

            /// Update the state of this vehicle at the current time.
            /// The vehicle system is provided the current driver inputs (throttle between 0 and 1, steering between -1 and +1,
            /// braking between 0 and 1). This version does not update any tires associated with the vehicle.
            virtual void Synchronize(double time,                       ///< [in] current time
                const DriverInputs& driver_inputs  ///< [in] current driver inputs
            ) override;

            /// Update the state of this vehicle at the current time.
            /// The vehicle system is provided the current driver inputs (throttle between 0 and 1, steering between -1 and +1,
            /// braking between 0 and 1), and a reference to the terrain system. If tires are associated with the vehicle
            /// wheels, their Synchronize method is invoked.
            virtual void Synchronize(double time,                        ///< [in] current time
                const DriverInputs& driver_inputs,  ///< [in] current driver inputs
                const ChTerrain& terrain            ///< [in] reference to the terrain system
            ) override;

            /// Return a JSON string with information on all modeling components in the vehicle system.
            /// These include bodies, shafts, joints, spring-damper elements, markers, etc.
            virtual std::string ExportComponentList() const override;

            virtual bool AddVehicleSystem(const std::string& name, std::shared_ptr<ChVehicleSystem> vehicle_system);
            virtual bool RemoveVehicleSystem(const std::string& name);

            virtual std::shared_ptr<ChVehicleSystem> GetVehicleSystem(const std::string& name);
            /// Create a wheeled vehicle from the provided JSON specification file.
            /// The vehicle is added to a newly created Chrono system which uses the specified contact formulation. If
            /// indicated, an associated powertrain and tires are created (if specified in the JSON file).
            ChWheeledVehicleAdvanced(const std::string& filename,
                ChContactMethod contact_method = ChContactMethod::NSC,
                bool create_powertrain = true,
                bool create_tires = true);

            /// Create a wheeled vehicle from the provided JSON specification file.
            /// The vehicle is added to the given Chrono system. If indicated, an associated powertrain and tires are created
            /// (if specified in the JSON file).
            ChWheeledVehicleAdvanced(ChSystem* system,
                const std::string& filename,
                bool create_powertrain = true,
                bool create_tires = true);

        protected:
            virtual void CreateSystems(const std::string& filename);

    

            /// Write output data for all modeling components in the wheeled vehicle system.
            virtual void WriteOutput(int frame, double time) const override;

            /// Checkpoint states of all modeling components in the wheeled vehicle system.
            virtual void SaveCheckpoint(ChCheckpoint& database) const override;

            /// Load states of all modeling components in the vehicle system from the specified checkpoint database.
            virtual void LoadCheckpoint(ChCheckpoint& database) override;

            std::map<std::string, std::shared_ptr<ChVehicleSystem>> m_vehicle_systems;
            
        };

        /// @} vehicle_wheeled

    }  // end namespace vehicle
}  // end namespace chrono

#endif
