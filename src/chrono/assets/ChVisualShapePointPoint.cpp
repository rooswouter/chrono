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

#include <cmath>

#include "chrono/assets/ChVisualShapePointPoint.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/physics/ChLinkMarkers.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkRevoluteSpherical.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChHydraulicActuator.h"

namespace chrono {

ChVisualShapePointPoint::ChVisualShapePointPoint() {
    SetMutable(true);
}

// Extract two positions from updater if it has any, and then update line geometry from these positions.
void ChVisualShapePointPoint::Update(ChObj* updater, const ChFrame<>& frame) {
    if (auto link_markers = dynamic_cast<ChLinkMarkers*>(updater)) {
        point1 = link_markers->GetMarker1()->GetAbsCoordsys().pos;
        point2 = link_markers->GetMarker2()->GetAbsCoordsys().pos;
    } else if (auto link_dist = dynamic_cast<ChLinkDistance*>(updater)) {
        point1 = link_dist->GetEndPoint1Abs();
        point2 = link_dist->GetEndPoint2Abs();
    } else if (auto link_rs = dynamic_cast<ChLinkRevoluteSpherical*>(updater)) {
        point1 = link_rs->GetPoint1Abs();
        point2 = link_rs->GetPoint2Abs();
    } else if (auto link_tsda = dynamic_cast<ChLinkTSDA*>(updater)) {
        point1 = link_tsda->GetPoint1Abs();
        point2 = link_tsda->GetPoint2Abs();
    } else if (auto link_mate = dynamic_cast<ChLinkMateGeneric*>(updater)) {
        point1 = link_mate->GetBody1()->TransformPointLocalToParent(link_mate->GetFrame1Rel().GetPos());
        point2 = link_mate->GetBody2()->TransformPointLocalToParent(link_mate->GetFrame2Rel().GetPos());
    } else if (auto link = dynamic_cast<ChLink*>(updater)) {
        point1 = link->GetBody1()->GetPos();
        point2 = link->GetBody2()->GetPos();
    } else if (auto actuator = dynamic_cast<ChHydraulicActuatorBase*>(updater)) {
        point1 = actuator->GetPoint1Abs();
        point2 = actuator->GetPoint2Abs();
    } else {
        return;
    }

    UpdateLineGeometry(frame.TransformPointParentToLocal(point1), frame.TransformPointParentToLocal(point2));
}

// Set line geometry as a segment between two end points.
void ChVisualShapeSegment::UpdateLineGeometry(const ChVector3d& endpoint1, const ChVector3d& endpoint2) {
    this->SetLineGeometry(std::static_pointer_cast<ChLine>(chrono_types::make_shared<ChLineSegment>(endpoint1, endpoint2)));
};

// Set line geometry as a coil between two end points.
void ChVisualShapeSpring::UpdateLineGeometry(const ChVector3d& endpoint1, const ChVector3d& endpoint2) {
    // If the visualization system doesn't need CPU-side geometry (for example - VSG generates
    // springs procedurally on GPU, irrlicht does its own thing), then no-op return to skip expensive ChLinePath
    // rebuilds of line vertices (which gets called every timestep on solvers that do full update!)
    // Primarily this impacts VSG because vsg is drawing the full 'mesh' of the lines every time, causing
    // cpu to gpu overhead, when we could just keep a gpu side procedural between points
    if (m_disable_geom_updates) {
        return;
    }

    auto linepath = chrono_types::make_shared<ChLinePath>();

    // Following part was copied from irrlicht::tools::DrawSpring()
    ChVector3d dist = endpoint2 - endpoint1;
    ChVector3d Vx, Vy, Vz;
    double length = dist.Length();
    ChMatrix33<> rel_matrix;
    rel_matrix.SetFromAxisX(dist, VECT_Y);
    ChCoordsys<> mpos(endpoint1, rel_matrix.GetQuaternion());

    double phaseA = 0;
    double phaseB = 0;
    double heightA = 0;
    double heightB = 0;

    for (int iu = 1; iu <= resolution; iu++) {
        phaseB = turns * CH_2PI * (double)iu / (double)resolution;
        heightB = length * ((double)iu / (double)resolution);
        ChVector3d V1(heightA, radius * std::cos(phaseA), radius * std::sin(phaseA));
        ChVector3d V2(heightB, radius * std::cos(phaseB), radius * std::sin(phaseB));

        auto segment = ChLineSegment(mpos.TransformPointLocalToParent(V1), mpos.TransformPointLocalToParent(V2));
        linepath->AddSubLine(segment);
        phaseA = phaseB;
        heightA = heightB;
    }

    this->SetLineGeometry(std::static_pointer_cast<ChLine>(linepath));
}

ChVisualShapePointPointMesh::ChVisualShapePointPointMesh(const std::string& filename,
                                                         double rest_length,
                                                         double radial_scale)
    : m_filename(filename), m_rest_length(rest_length), m_radial_scale(radial_scale) {
    m_mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    LoadMaterialsFromObj();
}

ChVisualShapePointPointMesh::ChVisualShapePointPointMesh(std::shared_ptr<ChTriangleMeshConnected> mesh,
                                                         double rest_length,
                                                         double radial_scale)
    : m_filename(mesh ? mesh->GetFileName() : ""),
      m_mesh(mesh),
      m_rest_length(rest_length),
      m_radial_scale(radial_scale) {
    LoadMaterialsFromObj();
}

double ChVisualShapePointPointMesh::GetRestLength() const {
    if (m_rest_length > 0)
        return m_rest_length;
    if (!m_mesh)
        return 1.0;
    double dz = m_mesh->GetBoundingBox().Size().z();
    return (dz > 0) ? dz : 1.0;
}

void ChVisualShapePointPointMesh::LoadMaterialsFromObj() {
    if (!m_mesh)
        return;
    // Reuse OBJ/MTL material loading implemented on ChVisualShapeTriangleMesh.
    ChVisualShapeTriangleMesh loader(m_mesh, true);
    for (unsigned int i = 0; i < loader.GetNumMaterials(); i++)
        AddMaterial(loader.GetMaterial((int)i));
}

void ChVisualShapeRotSpring::Update(ChObj* updater, const ChFrame<>& frame) {
    // Do nothing if not associated with an RSDA.
    auto rsda = dynamic_cast<ChLinkRSDA*>(updater);
    if (!rsda)
        return;

    // The asset frame for an RSDA is the frame on body1.
    auto linepath = chrono_types::make_shared<ChLinePath>();
    double del_angle = rsda->GetAngle() / m_resolution;
    ChVector3d V1(m_radius, 0, 0);
    for (int iu = 1; iu <= m_resolution; iu++) {
        double crt_angle = iu * del_angle;
        double crt_radius = m_radius - (iu * del_angle / CH_2PI) * (m_radius / 10);
        ChVector3d V2(crt_radius * std::cos(crt_angle), crt_radius * std::sin(crt_angle), 0);
        auto segment = ChLineSegment(V1, V2);
        linepath->AddSubLine(segment);
        V1 = V2;
    }

    this->SetLineGeometry(std::static_pointer_cast<ChLine>(linepath));
}

}  // end namespace chrono
