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
            : ChEngineShafts(name, dir_motor_block)
        {
        }

        ChEngineShaftsAdvanced::~ChEngineShaftsAdvanced()
        {
            
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
            auto engine = chrono_types::make_shared<ChShaftsThermalEngineAdvanced>();
            m_engine = engine;
            m_engine->Initialize(m_motorshaft, m_motorblock);
            sys->Add(m_engine);

            // The thermal engine requires a torque curve
            auto mTw = chrono_types::make_shared<ChFunctionInterp>();
            SetEngineTorqueMap(mTw);
            m_engine->SetTorqueCurve(mTw);
            engine->SetIdleSpeed(m_idle_speed);
            engine->SetMaxSpeed(m_max_speed);

            // Create an engine brake model to model engine losses due to inner friction, turbulence, etc.
            // Without this, the engine at 0% throttle in neutral position would rotate forever at constant speed.
            m_engine_losses = chrono_types::make_shared<ChShaftsThermalEngine>();
            m_engine_losses->Initialize(m_motorshaft, m_motorblock);
            sys->Add(m_engine_losses);

            // The engine brake model requires a torque curve
            auto mTw_losses = chrono_types::make_shared<ChFunctionInterp>();
            SetEngineLossesMap(mTw_losses);
            m_engine_losses->SetTorqueCurve(mTw_losses);
        }

        // -----------------------------------------------------------------------------
        void ChEngineShaftsAdvanced::Synchronize(double time, const DriverInputs& driver_inputs, double motorshaft_speed)
        {
            ChEngineShafts::Synchronize(time, driver_inputs, motorshaft_speed);
        }



    }  // end namespace vehicle
}  // end namespace chrono
