// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// EngineAdvanced Tests
//
// =============================================================================

#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/core/ChTimer.h"

#include "chrono_vehicle/advanced_systems/ChUtilsAdvancedSystems.h"
#include "chrono_vehicle/advanced_systems/ChEngineShaftsAdvanced.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
    #include "chrono_vsg/ChMouseOrbitZoomCameraVSGPlugin.h"
using namespace chrono::vsg3d;
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
using namespace chrono::postprocess;
#endif
#include "demos/vehicle/WheeledVehicleJSON.h"
//#include "../WheeledVehicleModels.h"

// =============================================================================

enum class TerrainType { FLAT, RIGID };
TerrainType terrain_type = TerrainType::RIGID;

// Terrain length (X direction)
double terrainLength = 800.0;

// Tire-terrain collision type (handling tire models)
ChTire::CollisionType tire_collision_type = ChTire::CollisionType::SINGLE_POINT;

// Include aerodynamic drag
bool include_aero_drag = false;

// Simulation step sizes
double step_size = 2e-3;

// End simulation time
double t_end = 100;

// Output
bool output = true;

enum class EngineTestsEnum
{
    ThrottleStepsNeutral = 0,
    Throttle25 = 1,
    Throttle50 = 2,
    Throttle100 = 3,
};
// =============================================================================

DriverInputs GetDriverInputs(EngineTestsEnum test_mode, WheeledVehicle &vehicle, double time);

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------
    // Create vehicle
    // --------------

    ChVector3d init_loc(-terrainLength / 2 + 5, 0, 0.7);
    ChVector3d path_start(-terrainLength / 2, 0, 0.5);
    ChVector3d path_end(+terrainLength / 2, 0, 0.5);

    // Select vehicle model (see VehicleModel.h)
    auto models = WheeledVehicleJSON::List();

    int num_models = (int)models.size();

    int which = 0;
    std::cout << "Options:\n";
    std::cout << "1: Throttle Steps in neutral" << std::endl;
    std::cout << "2: Throttle @ 25%" << std::endl;
    std::cout << "3: Throttle @ 50%" << std::endl;
    std::cout << "4: Throttle @ 100%" << std::endl;
    std::cout << "\nSelect test: ";
    std::cin >> which;
    std::cout << std::endl;
    ChClampValue(which, 1, 4);

    EngineTestsEnum test_mode = (EngineTestsEnum)(which - 1);
    // auto select ram (13)
    auto vehicle_model = models[0].first;

    // Register factories first
    RegisterAdvancedSystems();

    // Create the vehicle system
    WheeledVehicle vehicle(GetVehicleDataFile(vehicle_model->VehicleJSON()), ChContactMethod::SMC);
    vehicle.Initialize(ChCoordsys<>(ChVector3(0.0,0.0,0.65), QuatFromAngleZ(0.0)));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(GetVehicleDataFile(vehicle_model->EngineJSON()));
    auto transmission = ReadTransmissionJSON(GetVehicleDataFile(vehicle_model->TransmissionJSON()));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (unsigned int i = 0; i < vehicle.GetNumberAxles(); i++) {
        for (auto& wheel : vehicle.GetAxle(i)->GetWheels()) {
            auto tire = ReadTireJSON(GetVehicleDataFile(vehicle_model->TireJSON(i)));
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Containing system
    auto sys = vehicle.GetSystem();
    // Associate a collision system
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // -----------------------
    // Create output directory
    // -----------------------

    std::string out_dir = GetChronoOutputPath() + "ACCELERATION_ENGINEADVANCED";
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    out_dir = out_dir + "/" + vehicle_model->ModelName();
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // ------------------
    // Create the terrain
    // ------------------

    std::shared_ptr<ChTerrain> terrain;
    switch (terrain_type) {
        case TerrainType::RIGID:
        default: {
            auto rigid_terrain = chrono_types::make_shared<RigidTerrain>(vehicle.GetSystem());
            auto patch_mat = chrono_types::make_shared<ChContactMaterialSMC>();
            patch_mat->SetFriction(0.9f);
            patch_mat->SetRestitution(0.01f);
            patch_mat->SetYoungModulus(2e7f);
            patch_mat->SetPoissonRatio(0.3f);
            auto patch = rigid_terrain->AddPatch(patch_mat, ChCoordsys<>(), terrainLength, 5);
            patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
            patch->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 5);
            rigid_terrain->Initialize();
            terrain = rigid_terrain;
            break;
        }
        case TerrainType::FLAT: {
            auto flat_terrain = chrono_types::make_shared<FlatTerrain>(0, 0.9f);
            terrain = flat_terrain;
            break;
        }
    }

    // -----------------------------
    // Create path and driver system
    // -----------------------------

    auto path = StraightLinePath(path_start, path_end, 1);
    ChPathFollowerDriver driver(vehicle, path, "my_path", 1000.0);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // -----------------------------------------
    // Create the vehicle run-time visualization
    // -----------------------------------------

    std::string title = "Vehicle Acceleration Test";
    std::shared_ptr<ChVehicleVisualSystem> vis;
#ifdef CHRONO_VSG
    // Create the vehicle VSG interface
    auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis_vsg->SetWindowTitle(title);
    vis_vsg->AttachVehicle(&vehicle);
    vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), vehicle_model->CameraDistance(),
                            0.5);
    vis_vsg->SetWindowSize(1280, 800);
    vis_vsg->SetWindowPosition(100, 100);
    vis_vsg->EnableSkyTexture(SkyMode::DOME);
    vis_vsg->SetCameraAngleDeg(40);
    vis_vsg->SetLightIntensity(1.0f);
    vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis_vsg->EnableShadows();
    auto orbitPlugin = chrono_types::make_shared<chrono::vsg3d::ChMouseOrbitZoomCameraVSGPlugin>(chrono::CameraVerticalDir::Z);
    vis_vsg->AttachPlugin(orbitPlugin);

    vis_vsg->Initialize();

    vis = vis_vsg;
#endif

    // ---------------
    // Simulation loop
    // ---------------

    // Output file
    ChWriterCSV csv("\t");
    csv.Stream().setf(std::ios::scientific | std::ios::showpos);
    csv.Stream().precision(6);

    csv << "time";
    csv << "throttle";
    csv << "VehicleSpeed";
    csv << "EngineRPM";
    csv << "Engine Torque";
    csv << "CurrentTransmissionGear";
    csv << "Distance";
    csv << std::endl;

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);
    double last_speed = -1;

    // Record vehicle speed
    ChFunctionInterp speed_recorder;
    ChFunctionInterp dist_recorder;

    // Initialize simulation frame counter and simulation time
    double time = 0;
    bool done = false;

    ChTimer timer;
    timer.start();
    int mode = 0;   // 0 = paused, 1 = playing, 2 = single step
    ChVector3d prev_cam_target = vehicle.GetChassis()->GetPos();

    vehicle.EnableRealtime(true);

    while (true) {
        time = vehicle.GetSystem()->GetChTime();


        double speed = speed_filter.Add(vehicle.GetSpeed());
        double engine_speed = vehicle.GetEngine()->GetMotorSpeed();
        double engine_torque = vehicle.GetEngine()->GetOutputMotorshaftTorque();
        double dist = terrainLength / 2.0 + vehicle.GetPos().x();
        int gear_pos = vehicle.GetTransmission()->GetCurrentGear();


        last_speed = speed;

        // End simulation
        if (time >= t_end)
            break;

        // Render scene
        if (vis) {
            if (!vis->Run())
                break;
            const ChVector3d cur_cam_target = vehicle.GetChassis()->GetPos();
            const ChVector3d delta = cur_cam_target - prev_cam_target;

            if (delta.Length2() > 1e-20) {
                const ChVector3d cam_eye = vis->GetCameraPosition();
                vis->SetCameraPosition(cam_eye + delta);
                vis->SetCameraTarget(cur_cam_target);
                prev_cam_target = cur_cam_target;
            }

            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

       

        int key = orbitPlugin->GetKey();
        if (key == 'p') {
            if (mode == 1) {
                mode = 0;
            } else {
                mode = 1;
            }
        } else if (key == 'o') {
            mode = 2;
        }

        if (mode != 0) {
            if (!done) {
                speed_recorder.AddPoint(time, speed);
                dist_recorder.AddPoint(time, dist);

                if (time > 6 && std::abs((speed - last_speed) / step_size) < 2e-4) {
                    done = true;
                    timer.stop();
                    std::cout << "Simulation time: " << timer() << std::endl;
                    std::cout << "Maximum speed: " << speed << std::endl;
#ifdef CHRONO_POSTPROCESS
                    {
                        postprocess::ChGnuPlot gplot_speed(out_dir + "/speed.gpl");
                        gplot_speed.SetGrid();
                        gplot_speed.SetLabelX("time (s)");
                        gplot_speed.SetLabelY("speed (m/s)");
                        gplot_speed.Plot(speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
                    }
                    {
                        postprocess::ChGnuPlot gplot_dist(out_dir + "/dist.gpl");
                        gplot_dist.SetGrid();
                        gplot_dist.SetLabelX("time (s)");
                        gplot_dist.SetLabelY("dist (m)");
                        gplot_dist.Plot(dist_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
                    }
#endif
                }
            }

            // Driver inputs
            /*
            DriverInputs driver_inputs = driver.GetInputs();
            if (done) {
                driver_inputs.m_throttle = 0.1;
                driver_inputs.m_braking = 0.8;
            }
            */
            DriverInputs driver_inputs = GetDriverInputs(test_mode, vehicle, time);
            // Collect output
            if (output) {
                csv << time;
                csv << driver_inputs.m_throttle;
                csv << 3.6 * speed;
                csv << engine_speed * CH_RAD_S_TO_RPM;
                csv << engine_torque;
                csv << gear_pos;
                csv << dist;
                csv << std::endl;
            }
            
            if (mode == 2) mode = 0;
            // Update modules (process inputs from other modules)
            driver.Synchronize(time);
            terrain->Synchronize(time);
            vehicle.Synchronize(time, driver_inputs, *terrain);
            if (vis)
                vis->Synchronize(time, driver_inputs);

            // Advance simulation for one timestep for all modules
            driver.Advance(step_size);
            terrain->Advance(step_size);
            vehicle.Advance(step_size);
        }
    }

    if (output)
        csv.WriteToFile(out_dir + "/veh_engineadvanced.out");

    return 0;
}



void GetDriverInputsStep(DriverInputs &driver_inputs, WheeledVehicle& vehicle, double time) {
    // 25% steps, 2 seconds each

    driver_inputs.m_steering = 0.0;
    int step = (int)floor(time / 2.0);
    step = step % 8;
    driver_inputs.m_throttle = step <= 4 ? 0.25 * step : 1.0 - (0.25 * (step - 4));
    driver_inputs.m_braking = 1.0;
    driver_inputs.m_clutch = 0.0;

    vehicle.GetTransmission()->SetGear(0);
}

void GetDriverInputsThrottle(double throttle, DriverInputs& driver_inputs, WheeledVehicle& vehicle, double time) {
    driver_inputs.m_steering = 0.0;
    driver_inputs.m_throttle = throttle;
    driver_inputs.m_braking = 0.0;
    driver_inputs.m_clutch = 0.0;

    //vehicle.GetTransmission()->SetGear(1);
}

DriverInputs GetDriverInputs(EngineTestsEnum test_mode, WheeledVehicle& vehicle, double time) {
    DriverInputs driver_inputs;
    driver_inputs.m_steering = 0.0;
    driver_inputs.m_throttle = 0.0;
    driver_inputs.m_braking = 0.0;
    driver_inputs.m_clutch = 0.0;

    auto engineAdvanced = std::dynamic_pointer_cast<ChEngineShaftsAdvanced>(vehicle.GetEngine());
    if (time < 0.5) {
        return driver_inputs;
    }
    // Start engine first
    if (engineAdvanced->GetEngineState() != EngineState::RUNNING)
    {
        driver_inputs.m_ignition = IgnitionState::START;
        return driver_inputs;
    }
    driver_inputs.m_ignition = IgnitionState::ON;

    switch (test_mode) { 
        case EngineTestsEnum::ThrottleStepsNeutral:
            GetDriverInputsStep(driver_inputs, vehicle, time);
            break;
        case EngineTestsEnum::Throttle25:
            GetDriverInputsThrottle(0.25, driver_inputs, vehicle, time);
            break;
        case EngineTestsEnum::Throttle50:
            GetDriverInputsThrottle(0.5, driver_inputs, vehicle, time);
            break;
        case EngineTestsEnum::Throttle100:
            GetDriverInputsThrottle(1.0, driver_inputs, vehicle, time);
            break;
    }
    return driver_inputs;
}
