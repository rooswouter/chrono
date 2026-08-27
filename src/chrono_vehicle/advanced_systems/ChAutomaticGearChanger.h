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
// AutomaticGearChanger
// GearChanger for an Automatic Transmission
//
// =============================================================================

#ifndef CH_AUTOMATIC_GEAR_CHANGER_H
#define CH_AUTOMATIC_GEAR_CHANGER_H

#include "chrono_vehicle/ChApiVehicle.h"

#include "chrono_vehicle/advanced_systems/ChGearChanger.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Template for a Gear Changer .
class CH_VEHICLE_API ChAutomaticGearChanger : public ChGearChanger {
  public:
    /// Construct a shafts-based automatic transmission model.
    ChAutomaticGearChanger() {};
    ChAutomaticGearChanger(const rapidjson::Document& d);

    virtual ~ChAutomaticGearChanger() {};

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const { return "AutomaticGearChanger"; }

    virtual void Update(double time, std::shared_ptr<ChTransmission> transmission, const DriverInputs &driver_inputs, double engine_rpm);
  protected:
  private:
    virtual void Create(const rapidjson::Document& d);

    ChVectorN<double, 4> m_up_shift_coeff;
    ChVectorN<double, 4> m_down_shift_coeff;

    double m_gear_shift_latency = 0.0;
    double m_last_time_gearshift = 0.0;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
