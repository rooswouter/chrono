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
// Advanced engine model based on the EngineShafts model.
//  - Idle speed
//  - Throttle response
//
// =============================================================================

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/powertrain/ChShaftsThermalEngineAdvanced.h"
#include "chrono_vehicle/powertrain/EngineShaftsAdvanced.h"

namespace chrono {
    namespace vehicle {

        // -----------------------------------------------------------------------------
        // dir_motor_block specifies the direction of the motor block, i.e. the
        // direction of the crankshaft, in chassis local coords. This is needed because
        // ChShaftBodyRotation could transfer rolling torque to the chassis.
        // -----------------------------------------------------------------------------
        ChEngineShaftsAdvanced::ChEngineShaftsAdvanced(const std::string& name, const ChVector3d& dir_motor_block)
            : ChEngine(name), m_dir_motor_block(dir_motor_block)
        {
        }

        ChEngineShaftsAdvanced::~ChEngineShaftsAdvanced()
        {
            if (!IsInitialized())
                return;

            auto sys = m_engine->GetSystem();
            if (!sys)
                return;

            sys->Remove(m_motorblock);
            sys->Remove(m_motorblock_to_body);
            sys->Remove(m_engine);
            sys->Remove(m_motorshaft);
        }

        double ChEngineShaftsAdvanced::GetOutputMotorshaftTorque() const
        {
            return m_engine->GetReaction1();
        }

        // -----------------------------------------------------------------------------
        void ChEngineShaftsAdvanced::Initialize(std::shared_ptr<ChChassis> chassis)
        {
            ChEngine::Initialize(chassis);

            assert(chassis->GetBody()->GetSystem());
            auto sys = chassis->GetSystem();

            // Create the motorshaft which represents the crankshaft plus flywheel.
            m_motorshaft = chrono_types::make_shared<ChShaft>();
            m_motorshaft->SetInertia(GetMotorshaftInertia());
            sys->AddShaft(m_motorshaft);

            // Create the motor block.
            // ChShaftsThermalEngine connects this motor block to the motorshaft and applies the engine torque between them.
            m_motorblock = chrono_types::make_shared<ChShaft>();
            m_motorblock->SetInertia(GetMotorBlockInertia());
            sys->AddShaft(m_motorblock);

            // Create  a connection between the motor block and the 3D rigid body that represents the chassis.
            // This allows to get the effect of the car 'rolling' when the longitudinal engine accelerates suddenly.
            m_motorblock_to_body = chrono_types::make_shared<ChShaftBodyRotation>();
            m_motorblock_to_body->Initialize(m_motorblock, chassis->GetBody(), m_dir_motor_block);
            sys->Add(m_motorblock_to_body);

            // Create a thermal engine model between motor block and motorshaft (both receive the torque, but with opposite
            // sign).
            auto engine = chrono_types::make_shared<ChShaftsTorque>();
            m_engine = engine;
            m_engine->Initialize(m_motorshaft, m_motorblock);
            sys->Add(m_engine);

            // The torque curve lives inside the ChEngineShaftAdvanced
            m_torque_func = chrono_types::make_shared<ChFunctionInterp>();
            SetEngineTorqueMap(m_torque_func);


            // The shuffle torque curve lives inside the ChEngineShaftAdvanced
            m_shuffle_torque_func = chrono_types::make_shared<ChFunctionInterp>();
            SetEngineShuffleTorqueMap(m_shuffle_torque_func);


        }

        // -----------------------------------------------------------------------------
        void ChEngineShaftsAdvanced::Synchronize(double time, const DriverInputs& driver_inputs, double motorshaft_speed)
        {
            // Apply shaft speed
            m_motorshaft->SetPosDt(motorshaft_speed);

            // Update the throttle level in the thermal engine
            //m_engine->SetThrottle(driver_inputs.m_throttle);

            double mw = m_engine->GetRelativePosDt();
            bool error_backward = false;
            if (mw < 0)
                error_backward = true;
            else
                error_backward = false;
            
            double torque = 0.0;

#if 1
            // Throttle controls target speed of the engine
            // Control the engine speed to the desired speed based on the throttle position
            double throttle = driver_inputs.m_throttle;
            throttle = m_old_throttle + (throttle - m_old_throttle) * m_throttle_lag;
            m_old_throttle = throttle;
            throttle = std::clamp(throttle, 0.0, 1.0);
            double mw_desired = m_idle_speed + (m_max_speed - m_idle_speed) * throttle;
            double error = mw_desired - mw;

            // Going faster then throttle position wants, use shuffle torque

            if (error < 0.0) {
                torque = m_shuffle_torque_func->GetVal(mw);
            } else {
                torque = m_torque_func->GetVal(mw);
            }

            // get max available torque at current RPM

            // Crude P controller
            double modulated_T = std::min(0.1 * error, 1.0);
            torque *= modulated_T;
#else
            // Throttle controls torque

            torque = m_torque_func->GetVal(mw) ;
            torque *= driver_inputs.m_throttle;
            torque += m_shuffle_torque_func->GetVal(mw);
#endif
            m_engine->SetTorque(torque);

        }

        void ChEngineShaftsAdvanced::PopulateComponentList()
        {
            m_components.shafts.push_back(m_motorblock);
            m_components.shafts.push_back(m_motorshaft);

            m_components.couples.push_back(m_engine);
        }



    }  // end namespace vehicle
}  // end namespace chrono
