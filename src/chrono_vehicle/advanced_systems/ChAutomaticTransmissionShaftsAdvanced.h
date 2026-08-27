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
// NOT USED, remove!
//
// =============================================================================

#ifndef CH_SHAFTS_AUTOMATIC_TRANSMISSION_ADVANCED_H
#define CH_SHAFTS_AUTOMATIC_TRANSMISSION_ADVANCED_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionShafts.h"

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsGearbox.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChShaftsTorqueConverter.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

// Forward reference
class ChVehicle;

/// Template to extend a ChAutomaticTransmissionShafts with a ChGearChanger object .
class CH_VEHICLE_API ChAutomaticTransmissionShaftsAdvanced : public ChAutomaticTransmissionShafts {
  public:
    /// Construct a shafts-based automatic transmission model.
    ChAutomaticTransmissionShaftsAdvanced(const std::string& name);

    virtual ~ChAutomaticTransmissionShaftsAdvanced();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "AutomaticTransmissionShaftsAdvanced"; }


  protected:
  private:
    /// Initialize this transmission system by attaching it to an existing vehicle chassis and connecting the provided
    /// engine and driveline subsystems.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

    /// Synchronize the state of this transmission system at the current time.
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double motorshaft_torque,           ///< input engine torque
                             double driveshaft_speed             ///< input driveline speed
                             ) override;

    /// Perform any action required on a gear shift (the new gear and gear ratio are available).
    virtual void OnGearShift() override;

    /// Perform any action required on placing the transmission in neutral.
    virtual void OnNeutralShift() override;

    virtual void PopulateComponentList() override;

};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
