// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Advanced engine model based on the ChEngineShafts model.
//  - Idle speed
//  - Throttle response
//
// The losses are not implemented as shaft, but rather as a torque applied when throttle is 0.0
// Losses should not be added when engine is producing torque, as this leads to reduced performance. Hence second shaft is also not necessary
// =============================================================================

#ifndef CH_SHAFTS_ADVANCED_ENGINE_H
#define CH_SHAFTS_ADVANCED_ENGINE_H


#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChEngine.h"

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChShaftsThermalEngine.h"


namespace chrono {
    namespace vehicle {



        /// @addtogroup vehicle_powertrain
        /// @{

        enum class EngineState {
            OFF,
            STARTING,
            RUNNING,
            STALLED,
            DAMAGED
        };
        /// Advanced engine model based on the template ChEngine class
        class CH_VEHICLE_API ChEngineShaftsAdvanced : public ChEngine
        {
        public:
            virtual ~ChEngineShaftsAdvanced();

            /// Get the name of the vehicle subsystem template.
            virtual std::string GetTemplateName() const override { return "EngineShaftsAdvanced"; }

            /// Return the current engine speed.
            virtual double GetMotorSpeed() const override { return m_motorshaft->GetPosDt(); }

            /// Return the output engine torque.
            /// This is the torque passed to a transmission subsystem.
            virtual double GetOutputMotorshaftTorque() const override;

            /// Return the reaction torque on the chassis body.
            virtual double GetChassisReactionTorque() const override { return -m_motorblock_to_body->GetTorqueReactionOnShaft(); }
            /// Construct a shafts-based engine model.
            ChEngineShaftsAdvanced(const std::string& name, const ChVector3d& dir_motor_block = ChVector3d(1, 0, 0));

            /// Set inertia of the motor block.
            virtual double GetMotorBlockInertia() const = 0;

            /// Set inertia of the motorshaft (crankshaft + fly wheel).
            virtual double GetMotorshaftInertia() const = 0;

            /// Engine speed-torque map.
            virtual void SetEngineTorqueMap(std::shared_ptr<ChFunctionInterp2D>& map) = 0;


            virtual EngineState GetEngineState() const { return m_state; }

            /// Set Idle Speed [rad/s]
            void SetIdleSpeed(double idle_speed) { m_idle_speed = idle_speed; }
            void SetMaxSpeed(double max_speed) { m_max_speed = max_speed; }

          private:
            /// Initialize this engine system.
            virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

            /// Update the state of this engine system at the current time.
            /// Set the motorshaft speed to the provided value and update the throttle level for the engine.
            virtual void Synchronize(double time,                        ///< current time
                const DriverInputs& driver_inputs,  ///< current driver inputs
                double motorshaft_speed             ///< input transmission speed
            ) override;

            /// Advance the state of this engine system by the specified time step.
            /// Since the state of a EngineShafts is advanced as part of the vehicle state, this function does nothing.
            virtual void Advance(double step) override {}

            virtual void PopulateComponentList() override;

            virtual void UpdateEngineState(const DriverInputs& driver_inputs, double engine_speed);

            virtual double CalculateTorqueOff(double time, const DriverInputs& driver_inputs, double engine_speed);
            virtual double CalculateTorqueRunning(double time, const DriverInputs& driver_inputs, double engine_speed);
            virtual double CalculateTorqueStarting(double time, const DriverInputs& driver_inputs, double engine_speed);

            std::shared_ptr<ChShaft> m_motorblock;
            std::shared_ptr<ChShaftBodyRotation> m_motorblock_to_body;
            std::shared_ptr<ChShaftsTorque> m_engine;
            std::shared_ptr<ChShaft> m_motorshaft;  ///< shaft connection to the transmission

            ChVector3d m_dir_motor_block;


            double m_idle_speed = 0.0;      ///< The idle speed of the engine [rad/s]
            double m_max_speed = 0.0;       ///< The idle speed of the engine [rad/s]
            double m_stall_speed = -1.0;    ///< The stall speed of the engine [rad/s]. If negative, engine won't stall

            double m_old_throttle = 0.0;
            double m_throttle_lag = 0.01;

            std::shared_ptr<ChFunctionInterp2D> m_torque_func;  ///< Torque as a function of throttle (x) and engine speed (y)

            EngineState m_state = EngineState::OFF;
        };

        /// @} vehicle_powertrain

    }  // end namespace vehicle
}  // end namespace chrono

#endif
#pragma once
