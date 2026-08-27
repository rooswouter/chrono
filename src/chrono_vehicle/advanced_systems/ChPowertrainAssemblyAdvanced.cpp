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
// Implementation of a vehicle powertrain assembly advanced.
// Extends the base with a GearChanger object
//
// =============================================================================

#include "chrono_vehicle/advanced_systems/ChPowertrainAssemblyAdvanced.h"

namespace chrono {
namespace vehicle {

void ChPowertrainAssemblyAdvanced::Initialize(std::shared_ptr<ChChassis> chassis) {
    ChPowertrainAssembly::Initialize(chassis);
    m_gear_changer->Initialize(m_transmission);
}

void ChPowertrainAssemblyAdvanced::Synchronize(double time, const DriverInputs& driver_inputs, double driveshaft_speed) {
    m_gear_changer->Update(time, m_transmission, driver_inputs, m_engine->GetMotorSpeed() * CH_RAD_S_TO_RPM);
    ChPowertrainAssembly::Synchronize(time, driver_inputs, driveshaft_speed);
}

void ChPowertrainAssemblyAdvanced::Advance(double step) {
    ChPowertrainAssembly::Advance(step);
}

}  // end namespace vehicle
}  // end namespace chrono
