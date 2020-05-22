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
// Tow hitch model constructed from a JSON specification file.
//
// =============================================================================

/// TF TODO: 
///  * JSON parsing check
///  * sort out visualisation
///  * create python test
///  * integrate into next layer up
///  * check logging

#include "chrono_vehicle/wheeled_vehicle/towhitch/TowHitch.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TowHitch::TowHitch(const std::string& filename) : ChTowHitch("") {
    // Open and parse the input file
    Document d = ReadFileJSON(filename);
    Create(d);
}

TowHitch::TowHitch(const std::string& partname, const std::string& filename) : ChTowHitch(partname) {
    Document d = ReadFileJSON(filename);
    Create(d);
}

TowHitch::TowHitch(const std::string& partname, const Document& d) : ChTowHitch(partname) {
    Create(d);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TowHitch::Create(const Document& d) {

    if (d.IsNull()) {
        return;
    }

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    //assert(d.HasMember("Name"));

    //std::string name = d["Name"].GetString();
    std::string type = d["Type"].GetString();
    std::string subtype = d["Template"].GetString();

    assert(type.compare("TowHitch") == 0);


    // -------------------------------------------
    // Create the towhitch system
    // -------------------------------------------
    if (subtype.compare("Ball") == 0) {
        GetLog() << "Reading JSON towball data\n";

        m_hitch = chrono_types::make_shared<ChLinkLockSpherical>();
        m_hitch->SetNameString(m_name);

        if (d.HasMember("Limits")) {
            assert(d["Limits"].IsArray());
            assert(d["Limits"].Size() == 4 || d["Limits"].Size() == 6);
            m_hitch->GetLimit_Rx().SetMin(d["Limits"][0].GetDouble());
            m_hitch->GetLimit_Rx().SetMax(d["Limits"][1].GetDouble());
            m_hitch->GetLimit_Ry().SetMin(d["Limits"][2].GetDouble());
            m_hitch->GetLimit_Ry().SetMax(d["Limits"][3].GetDouble());
            if (d["Limits"].Size() == 6) {
                m_hitch->GetLimit_Rz().SetMin(d["Limits"][4].GetDouble());
                m_hitch->GetLimit_Rz().SetMax(d["Limits"][5].GetDouble());
            }
        }
    } else if (subtype.compare("Ball Receiver") == 0) {
        // do nothing.  The master ball will be the only joint created
    } else {
        /// TF: TODO: add 5th wheel hitch
        throw ChException("Hitch type not supported.");
    }

}

void TowHitch::Initialize(std::shared_ptr<ChChassis> chassis, const ChVector<>& hitch_location) {
    GetLog() << "Init tow hitch\n";
    m_chassis = chassis;
    m_hitch_location.Set(hitch_location);
}

void TowHitch::Couple(std::shared_ptr<ChTowHitch> slave) {
    // need to link to chassis here
    // documentation is a bit sketchy on how I attach this to a chassis element, and what happens if they are not
    // aligned (infinite speed relocation?), so this is still a bit of a guess

    assert(!m_hitched); // Can't hitch to more than one trailer
    if (!m_hitched) {
        m_hitch->Initialize(m_chassis->GetBody(), slave->GetChassisBody(), false, ChCoordsys<>(m_hitch_location),
                            ChCoordsys<>(slave->GetHitchLocation()));
        m_hitched = true;
        slave->SetHitched(m_hitch);
        // magic to link the two together in the simulation
        ChSystem* system = m_chassis->GetBody()->GetSystem();
        //system->Add(slave->GetChassisBody());
        system->Add(m_hitch);
    }
}

void TowHitch::AddVisualizationAssets(VisualizationType vis) {
    GetLog() << "Tow hitch visualisation\n";
    ChTowHitch::AddVisualizationAssets(vis);
}

}  // end namespace vehicle
}  // end namespace chrono
