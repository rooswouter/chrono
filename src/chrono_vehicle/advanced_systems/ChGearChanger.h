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
// GearChanger
// Template class for gear changer strategies
//
// =============================================================================

#ifndef CH_GEAR_CHANGER_H
#define CH_GEAR_CHANGER_H

#include "chrono_vehicle/ChApiVehicle.h"

#include "chrono_vehicle/ChTransmission.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{


/// Template for a Gear Changer .
class CH_VEHICLE_API ChGearChanger  {
  public:
    /// Construct a shafts-based automatic transmission model.
    ChGearChanger() {};

    virtual ~ChGearChanger() {};

    virtual void Initialize(std::shared_ptr<ChTransmission> transmission) {
        // Force the Transmission is manual mode if it is Automatic, since the GearChanger will be reposnsible for all gear changes
        if (transmission->IsAutomatic()) {
            transmission->asAutomatic()->SetShiftMode(ChAutomaticTransmission::ShiftMode::MANUAL);
        }
        
    }
    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const = 0;

    virtual void Update(double time, std::shared_ptr<ChTransmission> transmission, const DriverInputs& driver_inputs, double engine_rpm) = 0;
  protected:
  private:
    virtual void Create(const rapidjson::Document& d) = 0;

};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
