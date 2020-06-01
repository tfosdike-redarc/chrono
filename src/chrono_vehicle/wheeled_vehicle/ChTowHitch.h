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
// Class representing a wheeled vehicle hitch point. A ChHitch encapsulates a 
// joint.  Present implementation is very basic.
//
// =============================================================================

#ifndef CH_TOWHITCH_H
#define CH_TOWHITCH_H

#include <string>
#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a suspension subsystem.
class CH_VEHICLE_API ChTowHitch : public ChPart {
  public:
    ChTowHitch(const std::string& name  ///< [in] name of the subsystem
               );

    ~ChTowHitch(){}

    /// Get the location of the hitch relative to the chassis reference frame.
    const ChVector<> GetHitchLocation() const { return m_hitch_location; }
    const std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_chassis->GetBody(); }

    virtual std::string GetTemplateName() const override { return "TowHitch"; }

    /// Enable/disable output for this subsystem.
    void SetOutput(bool state) { m_output = state; }

    /// Initialize this hitch subsystem, by initializing its components.
    /// The hitch subsystem is initialized by attaching it to the specified chassis body at the specified location
    /// (with respect to and expressed in the reference frame of the chassis). 
    virtual void Initialize(std::shared_ptr<ChChassis> chassis, const ChVector<>& hitch_location) = 0;

    /// Synchronize this hitch subsystem.
    void Synchronize(){};

    void SetHitched(std::shared_ptr<ChLinkLock> other_hitch) {
        m_hitch = other_hitch;
        m_hitched = TRUE;
    }

    void Output(ChVehicleOutput& database) ;

    /// Add visualization assets to this subsystem, for the specified visualization mode.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove all visualization assets from this subsystem.
    virtual void RemoveVisualizationAssets() override;

    void ExportComponentList(rapidjson::Document& jsonDocument);

    virtual void Couple(std::shared_ptr<ChTowHitch> other) {}


  protected:
    std::shared_ptr<ChLinkLock> m_hitch;           ///< Hitch joint (does not exist for slave)
    std::shared_ptr<ChChassis> m_chassis;            ///< Chassis that this hitch is located on
    ChVector<> m_hitch_location;  ///< hitch location relative to chassis
    bool m_output=FALSE;                           ///< True to log hitch angles (for coupled master)
    bool m_hitched = FALSE;                        ///< True if hitch is coupled

};

typedef std::vector<std::shared_ptr<ChTowHitch> > ChTowHitchList;

}  // end namespace vehicle
}  // end namespace chrono

#endif
