// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Timothy Fosdike
// =============================================================================
//
// Tow hitch model constructed from a JSON specification file
//
// =============================================================================

#ifndef TOW_HITCH_H
#define TOW_HITCH_H

#include "chrono/core/ChCoordsys.h"

#include "chrono_vehicle/wheeled_vehicle/ChTowHitch.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled
/// @{

/// Wheeled vehicle model constructed from a JSON specification file.
class CH_VEHICLE_API TowHitch : public ChTowHitch {
  public:
    TowHitch(const std::string& filename);

    TowHitch(const std::string& partname, const std::string& filename);

    TowHitch(const std::string& partname, const rapidjson::Document& d);

    ~TowHitch() {}

    /// This will just store the parameters, but will not apply them
    void Initialize(std::shared_ptr<ChChassis> chassis, const ChVector<>& hitch_location);

    /// hitch two wheeleed vehicles together
    void Couple(std::shared_ptr<ChTowHitch> other);

    /// Add visualization of the tow hitch
    virtual void AddVisualizationAssets(VisualizationType vis) override;

  private:
    void Create(const rapidjson::Document& d);

};

/// @} vehicle_wheeled

}  // end namespace vehicle
}  // end namespace chrono

#endif
