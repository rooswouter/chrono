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
//
// Base class for a Vehicle System, a non physical system that has access to the
// vehicle components.
// ABS
// todo: rename to ChWheeledVehicleSystem?
// =============================================================================

#ifndef CH_VEHICLE_SYSTEM_H
#define CH_VEHICLE_SYSTEM_H

#include <ostream>

#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
    namespace vehicle {

        /// @addtogroup vehicle_wheeled
        /// @{


        class CH_VEHICLE_API ChVehicleSystem
        {
        public:
            /// Destructor.
            virtual ~ChVehicleSystem() {}

            /// Get the name of the vehicle system template.
            virtual std::string GetTemplateName() const { return "VehicleSystem"; }

            /// Initialize the given tire and attach it to the specified wheel.
            /// Optionally, specify tire visualization mode and tire-terrain collision detection method.
            /// This function should be called only after vehicle initialization.
            virtual void Initialize(ChWheeledVehicle *vehicle) { m_wheeled_vehicle = vehicle; }

            /// Update the state of this vehicle at the current time.
            /// The vehicle system is provided the current driver inputs (throttle between 0 and 1, steering between -1 and +1,
            /// braking between 0 and 1). This version does not update any tires associated with the vehicle.
            virtual void Synchronize(double time,                       ///< [in] current time
                DriverInputs& driver_inputs  ///< [in] current driver inputs
            ) = 0;

            virtual void Create(const std::string& filename)
            {
                // Open and parse the input file
                Document d;
                ReadFileJSON(filename, d);
                if (d.IsNull())
                    return;

                // Check that the given file is a transmission specification file.
                assert(d.HasMember("Type"));
                std::string type = d["Type"].GetString();
                assert(type.compare("VehicleSystem") == 0);

                Create(d);
            }
            virtual void Create(const rapidjson::Document& d) = 0;

            std::string GetName() { return m_name; };
            void SetName(const std::string& name) { m_name = name; }
        protected:
            ChWheeledVehicle *m_wheeled_vehicle = NULL;
            std::string m_name;  ///< system name
        };

        /// @} vehicle_wheeled

    }  // end namespace vehicle
}  // end namespace chrono

#endif
