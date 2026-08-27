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
// Implementation of a Automatic Gear Changer
//
// =============================================================================

#include "chrono_vehicle/advanced_systems/ChAutomaticGearChanger.h"

namespace chrono {
namespace vehicle {

	ChAutomaticGearChanger::ChAutomaticGearChanger(const rapidjson::Document& d)
	{
        Create(d);
    }

    void ChAutomaticGearChanger::Create(const rapidjson::Document& d) {
        assert(d["UpShift"].IsArray());        
        assert(d["UpShift"].Size() == 4);        
        assert(d["DownShift"].IsArray());        
        assert(d["DownShift"].Size() == 4);

        m_up_shift_coeff[0] = d["UpShift"][0u].GetDouble();
        m_up_shift_coeff[1] = d["UpShift"][1u].GetDouble();
        m_up_shift_coeff[2] = d["UpShift"][2u].GetDouble();
        m_up_shift_coeff[3] = d["UpShift"][3u].GetDouble();

        m_down_shift_coeff[0] = d["DownShift"][0u].GetDouble();
        m_down_shift_coeff[1] = d["DownShift"][1u].GetDouble();
        m_down_shift_coeff[2] = d["DownShift"][2u].GetDouble();
        m_down_shift_coeff[3] = d["DownShift"][3u].GetDouble();

        m_gear_shift_latency = d["Shift Latency"].GetDouble();
    }

	void ChAutomaticGearChanger::Update(double time, std::shared_ptr<ChTransmission> transmission, const DriverInputs& driver_inputs, double engine_rpm)
	{
        // All in RPM
        double up_shift_speed = m_up_shift_coeff[0] + m_up_shift_coeff[1] * driver_inputs.m_throttle + m_up_shift_coeff[2] * driver_inputs.m_throttle * driver_inputs.m_throttle +
                                m_up_shift_coeff[3] * driver_inputs.m_throttle * driver_inputs.m_throttle * driver_inputs.m_throttle;

        double down_shift_speed = m_down_shift_coeff[0] + m_down_shift_coeff[1] * driver_inputs.m_throttle +
                                  m_down_shift_coeff[2] * driver_inputs.m_throttle * driver_inputs.m_throttle +
                                  m_down_shift_coeff[3] * driver_inputs.m_throttle * driver_inputs.m_throttle * driver_inputs.m_throttle;

        if (time - m_last_time_gearshift < m_gear_shift_latency)
            return;

        if (engine_rpm > up_shift_speed) {
            transmission->ShiftUp();
            m_last_time_gearshift = time;
        } else if (engine_rpm < down_shift_speed) {
            transmission->ShiftDown();
            m_last_time_gearshift = time;
        }

	}


}  // end namespace vehicle
}  // end namespace chrono
