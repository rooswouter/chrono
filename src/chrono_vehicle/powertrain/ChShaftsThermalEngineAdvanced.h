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

#ifndef CHSHAFTHERMALENGINE_ADVANCED_H
#define CHSHAFTHERMALENGINE_ADVANCED_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono/functions/ChFunction.h"
#include "chrono/physics/ChShaftsThermalEngine.h"

namespace chrono {

    /// Class for defining a thermal engine between two one-degree-of-freedom parts/
    /// The first shaft is the 'crankshaft' to whom the torque is applied, the second is the motor block, that receives the
    /// negative torque.
    class CH_VEHICLE_API ChShaftsThermalEngineAdvanced : public ChShaftsThermalEngine
    {
    public:
        ChShaftsThermalEngineAdvanced();
        ChShaftsThermalEngineAdvanced(const ChShaftsThermalEngineAdvanced& other);
        ~ChShaftsThermalEngineAdvanced() {}

        /// "Virtual" copy constructor (covariant return type).
        virtual ChShaftsThermalEngineAdvanced* Clone() const override { return new ChShaftsThermalEngineAdvanced(*this); }

        /// Method to allow serialization of transient data to archives.
        virtual void ArchiveOut(ChArchiveOut& archive_out) override;

        /// Method to allow deserialization of transient data from archives.
        virtual void ArchiveIn(ChArchiveIn& archive_in) override;

        void SetIdleSpeed(double idle_speed) { m_idle_speed = idle_speed; }
        void SetMaxSpeed(double max_speed) { m_max_speed = max_speed; }
    private:
        virtual double ComputeTorque() override;

        double m_idle_speed = 0.0;
        double m_max_speed = 0.0;
    };

    CH_CLASS_VERSION(ChShaftsThermalEngineAdvanced, 0)

}  // end namespace chrono

#endif
