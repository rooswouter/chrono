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
// Authors: Wouter Roos
// =============================================================================
//
// Definition of a vehicle powertrain advanced assembly.
// Extends the original powertrain assembly with:
// -Gear changer
//
// =============================================================================

#ifndef CH_POWERTRAIN_ASSEMBLY_ADVANCED_H
#define CH_POWERTRAIN_ASSEMBLY_ADVANCED_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/advanced_systems/ChGearChanger.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Definition of a powertrain assembly advanced system.
/// Extends the base with a GearChanger object
class CH_VEHICLE_API ChPowertrainAssemblyAdvanced : public ChPowertrainAssembly {
  public:
    /// Construct a powertrain assembly with the specified engine and transmission subsystems.
    ChPowertrainAssemblyAdvanced(std::shared_ptr<ChEngine> engine, std::shared_ptr<ChTransmission> transmission, std::shared_ptr<ChGearChanger> gear_changer) : ChPowertrainAssembly (engine, transmission), m_gear_changer(gear_changer) {}

    virtual ~ChPowertrainAssemblyAdvanced() {}

    /// Initialize this powertrain system by attaching it to an existing vehicle chassis.
    void Initialize(std::shared_ptr<ChChassis> chassis);

    /// Synchronize the state of this powertrain system at the current time.
    void Synchronize(double time,                        ///< current time
                     const DriverInputs& driver_inputs,  ///< current driver inputs
                     double driveshaft_speed             ///< input driveline speed
    );

    /// Advance the state of this powertrain system by the specified time step.
    void Advance(double step);

  private:
    std::shared_ptr<ChGearChanger> m_gear_changer;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
