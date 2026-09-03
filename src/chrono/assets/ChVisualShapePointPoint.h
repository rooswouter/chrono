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

#ifndef CH_VISUAL_SHAPE_POINTPOINT_H
#define CH_VISUAL_SHAPE_POINTPOINT_H

#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Base class for visualization of some deformable line shape
/// between two moving points related to the parent ChPhysicsItem.
/// (at this point, only ChLink and its derivatives are supported.)
/// NOTE: An instance of this class should not be shared among multiple ChPhysicsItem instances.
/// Otherwise drawing may broken since each physics item will try to update
/// geometry of the line and causes race conditions.

class ChApi ChVisualShapePointPoint : public ChVisualShapeLine {
  public:
    ChVisualShapePointPoint();

    // Update the underlying line geometry and set current locations of the end points.
    virtual void Update(ChObj* updater, const ChFrame<>& coords) override;

    const ChVector3d& GetPoint1Abs() const { return point1; }
    const ChVector3d& GetPoint2Abs() const { return point2; }

  private:
    // Update underlying line geometry from given two endpoints.
    // This method will be called on Update() call and should be implemented by derived classes.
    virtual void UpdateLineGeometry(const ChVector3d& endpoint1, const ChVector3d& endpoint2) = 0;

    ChVector3d point1;  ///< location of 1st end point (in global frame)
    ChVector3d point2;  ///< location of 2nd end point (in global frame)
};

/// Shape for visualizing a line segment between two moving points related to the parent ChPhysicsItem.
/// An instance of this class should not be shared among multiple ChPhysicsItem instances. Otherwise drawing may broken
/// since each physics item will try to update geometry of the line and causes race conditions.
class ChApi ChVisualShapeSegment : public ChVisualShapePointPoint {
  private:
    /// Set line geometry as segment between two end points (assumed in local frame).
    virtual void UpdateLineGeometry(const ChVector3d& endpoint1, const ChVector3d& endpoint2) override;
};

/// Shape for visualizing a coil spring between two moving points related to the parent ChPhysicsItem.
/// An instance of this class should not be shared among multiple ChPhysicsItem instances. Otherwise drawing may broken
/// since each physics item will try to update geometry of the line and causes race conditions.
class ChApi ChVisualShapeSpring : public ChVisualShapePointPoint {
  public:
    ChVisualShapeSpring(double mradius = 0.05, int mresolution = 65, double mturns = 5.) : radius(mradius), turns(mturns), resolution(mresolution) {}
    double GetRadius() const { return radius; }
    size_t GetResolution() const { return resolution; }
    double GetTurns() const { return turns; }

    /// Disable CPU-side visual geometry updates (for visualization systems that generate geometry on GPU)
    void SetGeometryUpdatesDisabled(bool disable) { m_disable_geom_updates = disable; }

  private:
    /// Set line geometry as coil between two end points (assumed in local frame).
    virtual void UpdateLineGeometry(const ChVector3d& endpoint1, const ChVector3d& endpoint2) override;

  private:
    double radius;
    double turns;
    size_t resolution;
    bool m_disable_geom_updates = false;  ///< Skip CPU geometry updates
};

/// Shape for visualizing a triangle mesh stretched between two moving points.
/// Visualization systems (e.g. Chrono::VSG) instance the mesh once and update a transform each frame from the two
/// endpoints. The mesh is assumed to be modeled along the Z axis, with its origin at the midpoint of the rest pose.
/// \a rest_length is the mesh length along Z used for axial scaling (if <= 0, the mesh AABB size in Z is used).
/// An instance of this class should not be shared among multiple ChPhysicsItem instances.
class ChApi ChVisualShapePointPointMesh : public ChVisualShapePointPoint {
  public:
    ChVisualShapePointPointMesh(const std::string& filename, double rest_length = 0, double radial_scale = 1);
    ChVisualShapePointPointMesh(std::shared_ptr<ChTriangleMeshConnected> mesh,
                                double rest_length = 0,
                                double radial_scale = 1);

    std::shared_ptr<ChTriangleMeshConnected> GetMesh() const { return m_mesh; }
    const std::string& GetFilename() const { return m_filename; }

    /// Rest length used for axial (Z) scaling. If unset, inferred from the mesh bounding box.
    double GetRestLength() const;
    void SetRestLength(double length) { m_rest_length = length; }

    double GetRadialScale() const { return m_radial_scale; }
    void SetRadialScale(double scale) { m_radial_scale = scale; }

  private:
    virtual void UpdateLineGeometry(const ChVector3d& endpoint1, const ChVector3d& endpoint2) override {}

    void LoadMaterialsFromObj();

    std::string m_filename;
    std::shared_ptr<ChTriangleMeshConnected> m_mesh;
    double m_rest_length;
    double m_radial_scale;
};

/// Shape representing a rotational spring.
class ChApi ChVisualShapeRotSpring : public ChVisualShapeLine {
  public:
    ChVisualShapeRotSpring(double radius, int resolution = 65) : m_radius(radius), m_resolution(resolution) {}

    virtual void Update(ChObj* updater, const ChFrame<>& coords) override;

  private:
    double m_radius;
    size_t m_resolution;
};

/// @} chrono_assets

}  // end namespace chrono

#endif
