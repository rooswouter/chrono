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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHFUNCT_INTERP2D_H
#define CHFUNCT_INTERP2D_H

#include <map>
#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Bilinear interpolation of a scalar table z = f(x, y).
/// This class is a 2D counterpart to ChFunctionInterp. It is not derived from ChFunction, which is strictly 1D.
///
/// Points must form a complete rectangular grid: every combination of unique x and unique y knots must be present.
/// Knot spacing may be uneven. Scattered or missing cells are not supported.
class ChApi ChFunctionInterp2D {
  public:
    ChFunctionInterp2D() {}
    ChFunctionInterp2D(const ChFunctionInterp2D& other);
    ~ChFunctionInterp2D() {}

    /// Add a point to the table.
    /// By default, adding a point with an (x, y) pair that already exists in the table will lead to an exception.
    /// If \a overwrite_if_existing is set to \c true, the existing point will be overwritten instead.
    void AddPoint(double x, double y, double z, bool overwrite_if_existing = false);

    /// Clear internal stored data.
    void Reset();

    /// Return the interpolated value z = f(x, y).
    /// Throws std::invalid_argument if the table is not a complete rectangular grid.
    double GetVal(double x, double y) const;

    /// Return df/dx of the local bilinear (or 1D) patch.
    /// Outside the domain, the derivative is zero unless extrapolation is enabled.
    double GetDerX(double x, double y) const;

    /// Return df/dy of the local bilinear (or 1D) patch.
    /// Outside the domain, the derivative is zero unless extrapolation is enabled.
    double GetDerY(double x, double y) const;

    /// Return the smallest value of x in the table.
    double GetStartX() const;

    /// Return the biggest value of x in the table.
    double GetEndX() const;

    /// Return the smallest value of y in the table.
    double GetStartY() const;

    /// Return the biggest value of y in the table.
    double GetEndY() const;

    /// Enable linear extrapolation.
    /// If enabled, the function continues the nearest bilinear cell outside the domain.
    /// If disabled (default), samples outside the domain hold the nearest boundary value.
    void SetExtrapolate(bool extrapolate) { m_extrapolate = extrapolate; }

    /// Retrieve the underlying table of points.
    const std::map<double, std::map<double, double>>& GetTable() const { return m_table; }

    /// Method to allow serialization of transient data to archives.
    void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    void ArchiveIn(ChArchiveIn& archive_in);

  private:
    void RebuildGrid() const;
    void EnsureGrid() const;

    /// Locate the cell index i such that knots[i] <= t <= knots[i+1] (clamped to a valid cell).
    int FindCell(const std::vector<double>& knots, double t, int& last) const;

    double EvalBilinear(double x, double y, bool* out_x, bool* out_y) const;

    std::map<double, std::map<double, double>> m_table;  ///< nested map: x -> (y -> z)

    mutable bool m_grid_dirty = true;
    mutable bool m_grid_complete = false;
    mutable std::vector<double> m_x;  ///< sorted unique x knots
    mutable std::vector<double> m_y;  ///< sorted unique y knots
    mutable ChMatrixDynamic<> m_z;    ///< z(i, j) at (m_x[i], m_y[j])
    mutable int m_last_ix = 0;        ///< last bracketing x-cell index
    mutable int m_last_iy = 0;        ///< last bracketing y-cell index

    bool m_extrapolate = false;  ///< enable linear extrapolation for out-of-range values
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionInterp2D, 0)

}  // end namespace chrono

#endif
