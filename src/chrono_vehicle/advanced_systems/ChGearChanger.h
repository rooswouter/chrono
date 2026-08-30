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

#include "chrono_vehicle/advanced_systems/ChWheeledVehicleAdvanced.h"
#include "chrono_vehicle/advanced_systems/ChVehicleSystem.h"
#include "chrono_vehicle/ChTransmission.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{


/// Template for a Gear Changer .
class CH_VEHICLE_API ChGearChanger : public ChVehicleSystem
{
  public:
    /// Construct a shafts-based automatic transmission model.
    ChGearChanger() {};

    virtual ~ChGearChanger() {};

    virtual void Initialize(ChWheeledVehicle* vehicle)
    {
        ChVehicleSystem::Initialize(vehicle);
        m_transmission = vehicle->GetTransmission();
        // Transmission normally gets set later, so call initialize again if m_transmission is invalid.
        if (!m_transmission) {
            return;
        }
        // Force the Transmission is manual mode if it is Automatic, since the GearChanger will be responsible for all gear changes
        if (m_transmission->IsAutomatic()) {
            m_transmission->asAutomatic()->SetShiftMode(ChAutomaticTransmission::ShiftMode::MANUAL);
        }
        m_engine = vehicle->GetEngine();
    }

    virtual void Synchronize(double time, DriverInputs& driver_inputs)
    {
        if (!m_transmission) {
            Initialize(m_wheeled_vehicle);
        }
    };

protected:
    std::shared_ptr<ChTransmission> m_transmission = nullptr;
    std::shared_ptr<ChEngine> m_engine = nullptr;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
