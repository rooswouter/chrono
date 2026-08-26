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

            
            double mw = m_engine->GetRelativePosDt();
            bool error_backward = false;
            if (mw < 0)
                error_backward = true;
            else
                error_backward = false;

            UpdateEngineState(driver_inputs, mw);

            double torque = 0.0;
            switch (m_state) 
            { 
                case EngineState::OFF:
                case EngineState::STALLED:
                    torque = CalculateTorqueOff(time, driver_inputs, mw);
                    break;
                case EngineState::RUNNING:
                    torque = CalculateTorqueRunning(time, driver_inputs, mw);
                    break;
                case EngineState::STARTING:
                    torque = CalculateTorqueStarting(time, driver_inputs, mw);
                    break;
            }
            m_engine->SetTorque(torque);
        }

        double ChEngineShaftsAdvanced::CalculateTorqueOff(double time, const DriverInputs& driver_inputs, double engine_speed)
        {
            // @todo control speed to 0.0?
            return 0.0;
        }

        double ChEngineShaftsAdvanced::CalculateTorqueRunning(double time, const DriverInputs& driver_inputs, double engine_speed)
        {
            if (m_stall_speed > 0.0 && engine_speed < m_stall_speed) {
                m_state = EngineState::STALLED;
               
            }

            double torque = m_torque_func->GetVal(driver_inputs.m_throttle, engine_speed);
            // Do not overrev the engine
            if (engine_speed > m_max_speed) {
                torque = std::min(torque, 0.0);
            } else if (engine_speed < m_idle_speed) {
                torque = m_torque_func->GetVal(0.2, engine_speed);
            }
            return torque;
        }

        double ChEngineShaftsAdvanced::CalculateTorqueStarting(double time, const DriverInputs& driver_inputs, double engine_speed)
        {
            // Control to idle speed, if we reach idle speed, set state to Running
            if (engine_speed > m_idle_speed) {
                m_state = EngineState::RUNNING;
            }
            return m_torque_func->GetVal(0.2, engine_speed);
        }

        void ChEngineShaftsAdvanced::PopulateComponentList()
        {
            m_components.shafts.push_back(m_motorblock);
            m_components.shafts.push_back(m_motorshaft);

            m_components.couples.push_back(m_engine);
        }


        void ChEngineShaftsAdvanced::UpdateEngineState(const DriverInputs& driver_inputs, double engine_speed)
        {
            switch (driver_inputs.ignition) { 
                case IgnitionState::OFF:
                    m_state = EngineState::OFF;
                    break;

                case IgnitionState::ACCESSORIES:
                    // Ignore for now, should enable power to vehicle systems
                    break;
                case IgnitionState::ON:
                    switch (m_state) {
                        case EngineState::STARTING:
                            m_state = EngineState::RUNNING;
                            break;
                        case EngineState::OFF:
                        case EngineState::STALLED:
                        case EngineState::DAMAGED:
                        case EngineState::RUNNING:
                        default:
                            break;
                    }
                    break;
                case IgnitionState::START:
                    switch (m_state) {
                        case EngineState::OFF:
                            m_state = EngineState::STARTING;
                            break;
                        case EngineState::RUNNING:
                            // Make starter motor noise?
                            break;
                        case EngineState::STARTING:
                        case EngineState::STALLED:
                        case EngineState::DAMAGED:
                        default:
                            break;
                    }
                    break;

            }



        }

    }  // end namespace vehicle
}  // end namespace chrono
