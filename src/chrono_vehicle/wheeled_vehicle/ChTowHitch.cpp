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

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/ChTowHitch.h"
#include "chrono/assets/ChSphereShape.h"

namespace chrono {
namespace vehicle {

ChTowHitch::ChTowHitch(const std::string& name) : ChPart(name) {}

void ChTowHitch::Output(ChVehicleOutput& database)  {
    if (!m_output || !m_hitched)
        return;
    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_hitch);
    database.WriteJoints(joints);
}

// FIXME: cannot show chassis and towhitch visualisation concurrently.
// TowHitch should be add accessor for visualisation primitives, to be called from chassis visualisation
void ChTowHitch::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    auto sphere_shape = chrono_types::make_shared<ChSphereShape>();
    sphere_shape->GetSphereGeometry().rad = 0.05;
    sphere_shape->SetColor(ChColor(1, 0, 0));
    sphere_shape->Pos = m_hitch_location;
    m_chassis->GetBody()->AddAsset(sphere_shape);
}

void ChTowHitch::RemoveVisualizationAssets() {
    // leave removing the hitch visualisation to when the rest of the chassis visualisation is cleared 
    //m_chassis->GetBody()->GetAssets().clear();
}

void ChTowHitch::ExportComponentList(rapidjson::Document& jsonDocument) {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChLink>> links;
    links.push_back(m_hitch);
    ChPart::ExportJointList(jsonDocument, links);
}


}  // end namespace vehicle
}  // end namespace chrono
