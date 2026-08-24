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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChShaft.h"
#include "chrono_vehicle/powertrain/ChShaftsThermalEngineAdvanced.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

    // Register into the object factory, to enable run-time dynamic creation and persistence
    CH_FACTORY_REGISTER(ChShaftsThermalEngineAdvanced)

    ChShaftsThermalEngineAdvanced::ChShaftsThermalEngineAdvanced()
    {

    }

    ChShaftsThermalEngineAdvanced::ChShaftsThermalEngineAdvanced(const ChShaftsThermalEngineAdvanced& other) : ChShaftsThermalEngine(other)
    {
        m_idle_speed = other.m_idle_speed;
    }

    double ChShaftsThermalEngineAdvanced::ComputeTorque()
    {
        // COMPUTE THE TORQUE HERE!
        double mw = GetRelativePosDt();

        if (mw < 0)
            error_backward = true;
        else
            error_backward = false;

        // Control the engine speed to the desired speed based on the throttle position
        double mw_desired = m_idle_speed + (m_max_speed - m_idle_speed) * throttle;
        double error = mw_desired - mw;
        // Going faster then throttle position wants
        if (error < 0.0) {
            return 0.0;
        }

        // get max available torque at current RPM
        double mT = Tw->GetVal(mw);
        printf("mw_desired = %f, throttle = %f, error = %f = %f - %f, mT = %f\n", mw_desired, throttle, error, mw_desired, mw, mT);

        // Crude P controller
        double modulated_T = std::min(0.1 * error, 1.0);
        modulated_T *= mT;

        return modulated_T;
    }

    void ChShaftsThermalEngineAdvanced::ArchiveOut(ChArchiveOut& archive_out)
    {
        // version number
        archive_out.VersionWrite<ChShaftsThermalEngineAdvanced>();

        // serialize parent class
        ChShaftsThermalEngine::ArchiveOut(archive_out);

        // serialize all member data:
        archive_out << CHNVP(m_idle_speed);
    }

    /// Method to allow de serialization of transient data from archives.
    void ChShaftsThermalEngineAdvanced::ArchiveIn(ChArchiveIn& archive_in)
    {
        // version number
        /*int version =*/archive_in.VersionRead<ChShaftsThermalEngineAdvanced>();

        // deserialize parent class:
        ChShaftsThermalEngine::ArchiveIn(archive_in);

        // deserialize all member data:
        archive_in >> CHNVP(m_idle_speed);
    }

}  // end namespace chrono