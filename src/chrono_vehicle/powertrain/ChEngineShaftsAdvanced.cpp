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
            m_torque_func = chrono_types::make_shared<ChFunctionInterp2D>();
            SetEngineTorqueMap(m_torque_func);


           

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
            
            double torque = m_torque_func->GetVal(driver_inputs.m_throttle, mw);
            // Do not overrev the engine
            if (mw > m_max_speed) {
                torque = std::min(torque, 0.0);
            } else if (mw < m_idle_speed) {
                torque = m_torque_func->GetVal(0.2, mw);
            }
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
