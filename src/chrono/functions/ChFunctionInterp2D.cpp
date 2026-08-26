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

#include <algorithm>
#include <set>
#include <stdexcept>

#include "chrono/core/ChClassFactory.h"
#include "chrono/functions/ChFunctionInterp2D.h"

namespace chrono {

CH_FACTORY_REGISTER(ChFunctionInterp2D)

namespace {

double LinearInterp(double t0, double v0, double t1, double v1, double t, bool extrapolate) {
    if (t1 == t0) {
        return v0;
    }
    if (!extrapolate) {
        if (t <= t0) {
            return v0;
        }
        if (t >= t1) {
            return v1;
        }
    }
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0);
}

double LinearDer(double t0, double v0, double t1, double v1, double t, bool extrapolate) {
    if (t1 == t0) {
        return 0.0;
    }
    if (!extrapolate && (t < t0 || t > t1)) {
        return 0.0;
    }
    return (v1 - v0) / (t1 - t0);
}

}  // namespace

ChFunctionInterp2D::ChFunctionInterp2D(const ChFunctionInterp2D& other) {
    m_table = other.m_table;
    m_extrapolate = other.m_extrapolate;
    m_grid_dirty = true;
    m_grid_complete = false;
    m_last_ix = 0;
    m_last_iy = 0;
}

void ChFunctionInterp2D::AddPoint(double x, double y, double z, bool overwrite_if_existing) {
    auto& y_row = m_table[x];
    auto ret = y_row.emplace(y, z);

    if (!ret.second) {
        if (overwrite_if_existing) {
            ret.first->second = z;
        } else {
            throw std::invalid_argument("Point already exists and overwrite flag was not set.");
        }
    }

    m_grid_dirty = true;
}

void ChFunctionInterp2D::Reset() {
    m_table.clear();
    m_x.clear();
    m_y.clear();
    m_z.resize(0, 0);
    m_grid_dirty = true;
    m_grid_complete = false;
    m_last_ix = 0;
    m_last_iy = 0;
}

void ChFunctionInterp2D::RebuildGrid() const {
    m_x.clear();
    m_y.clear();
    m_grid_complete = false;
    m_last_ix = 0;
    m_last_iy = 0;

    if (m_table.empty()) {
        m_z.resize(0, 0);
        m_grid_complete = true;
        m_grid_dirty = false;
        return;
    }

    std::set<double> y_set;
    for (const auto& x_row : m_table) {
        m_x.push_back(x_row.first);
        for (const auto& y_col : x_row.second) {
            y_set.insert(y_col.first);
        }
    }
    m_y.assign(y_set.begin(), y_set.end());

    const int nx = static_cast<int>(m_x.size());
    const int ny = static_cast<int>(m_y.size());

    for (const auto& x_row : m_table) {
        if (static_cast<int>(x_row.second.size()) != ny) {
            m_z.resize(0, 0);
            m_grid_dirty = false;
            return;
        }
        for (double y : m_y) {
            if (x_row.second.find(y) == x_row.second.end()) {
                m_z.resize(0, 0);
                m_grid_dirty = false;
                return;
            }
        }
    }

    m_z.resize(nx, ny);
    for (int i = 0; i < nx; ++i) {
        const auto& y_row = m_table.at(m_x[i]);
        for (int j = 0; j < ny; ++j) {
            m_z(i, j) = y_row.at(m_y[j]);
        }
    }

    m_grid_complete = true;
    m_grid_dirty = false;
}

void ChFunctionInterp2D::EnsureGrid() const {
    if (m_grid_dirty) {
        RebuildGrid();
    }
    if (!m_grid_complete) {
        throw std::invalid_argument("ChFunctionInterp2D table is not a complete rectangular grid.");
    }
}

int ChFunctionInterp2D::FindCell(const std::vector<double>& knots, double t, int& last) const {
    const int n = static_cast<int>(knots.size());
    if (n < 2) {
        last = 0;
        return 0;
    }

    if (t <= knots.front()) {
        last = 0;
        return 0;
    }
    if (t >= knots.back()) {
        last = n - 2;
        return n - 2;
    }

    if (last < 0 || last > n - 2 || !(t >= knots[last] && t <= knots[last + 1])) {
        auto it = std::upper_bound(knots.begin(), knots.end(), t);
        last = static_cast<int>(std::distance(knots.begin(), it) - 1);
        if (last < 0) {
            last = 0;
        }
        if (last > n - 2) {
            last = n - 2;
        }
    }

    return last;
}

double ChFunctionInterp2D::EvalBilinear(double x, double y, bool* out_x, bool* out_y) const {
    const int nx = static_cast<int>(m_x.size());
    const int ny = static_cast<int>(m_y.size());

    const bool x_out = (x < m_x.front() || x > m_x.back());
    const bool y_out = (y < m_y.front() || y > m_y.back());
    if (out_x) {
        *out_x = x_out;
    }
    if (out_y) {
        *out_y = y_out;
    }

    const int i = FindCell(m_x, x, m_last_ix);
    const int j = FindCell(m_y, y, m_last_iy);

    const double x0 = m_x[i];
    const double x1 = (nx > 1) ? m_x[i + 1] : m_x[i];
    const double y0 = m_y[j];
    const double y1 = (ny > 1) ? m_y[j + 1] : m_y[j];

    double tx = (x1 != x0) ? (x - x0) / (x1 - x0) : 0.0;
    double ty = (y1 != y0) ? (y - y0) / (y1 - y0) : 0.0;

    if (!m_extrapolate) {
        tx = std::clamp(tx, 0.0, 1.0);
        ty = std::clamp(ty, 0.0, 1.0);
    }

    const int i1 = (nx > 1) ? i + 1 : i;
    const int j1 = (ny > 1) ? j + 1 : j;

    const double z00 = m_z(i, j);
    const double z10 = m_z(i1, j);
    const double z01 = m_z(i, j1);
    const double z11 = m_z(i1, j1);

    return (1.0 - tx) * (1.0 - ty) * z00 + tx * (1.0 - ty) * z10 + (1.0 - tx) * ty * z01 + tx * ty * z11;
}

double ChFunctionInterp2D::GetVal(double x, double y) const {
    EnsureGrid();

    if (m_x.empty()) {
        return 0.0;
    }

    const int nx = static_cast<int>(m_x.size());
    const int ny = static_cast<int>(m_y.size());

    // Single sample: constant.
    if (nx == 1 && ny == 1) {
        return m_z(0, 0);
    }

    // Degenerate: 1D along y.
    if (nx == 1) {
        const int j = FindCell(m_y, y, m_last_iy);
        const int j1 = (ny > 1) ? j + 1 : j;
        return LinearInterp(m_y[j], m_z(0, j), m_y[j1], m_z(0, j1), y, m_extrapolate);
    }

    // Degenerate: 1D along x.
    if (ny == 1) {
        const int i = FindCell(m_x, x, m_last_ix);
        const int i1 = (nx > 1) ? i + 1 : i;
        return LinearInterp(m_x[i], m_z(i, 0), m_x[i1], m_z(i1, 0), x, m_extrapolate);
    }

    return EvalBilinear(x, y, nullptr, nullptr);
}

double ChFunctionInterp2D::GetDerX(double x, double y) const {
    EnsureGrid();

    if (m_x.empty()) {
        return 0.0;
    }

    const int nx = static_cast<int>(m_x.size());
    const int ny = static_cast<int>(m_y.size());

    if (nx == 1) {
        return 0.0;
    }

    if (ny == 1) {
        const int i = FindCell(m_x, x, m_last_ix);
        return LinearDer(m_x[i], m_z(i, 0), m_x[i + 1], m_z(i + 1, 0), x, m_extrapolate);
    }

    bool out_x = false;
    bool out_y = false;
    EvalBilinear(x, y, &out_x, &out_y);
    if (!m_extrapolate && out_x) {
        return 0.0;
    }

    const int i = FindCell(m_x, x, m_last_ix);
    const int j = FindCell(m_y, y, m_last_iy);
    const double x0 = m_x[i];
    const double x1 = m_x[i + 1];
    const double y0 = m_y[j];
    const double y1 = m_y[j + 1];
    double ty = (y1 != y0) ? (y - y0) / (y1 - y0) : 0.0;
    if (!m_extrapolate) {
        ty = std::clamp(ty, 0.0, 1.0);
    }

    const double z00 = m_z(i, j);
    const double z10 = m_z(i + 1, j);
    const double z01 = m_z(i, j + 1);
    const double z11 = m_z(i + 1, j + 1);

    return ((1.0 - ty) * (z10 - z00) + ty * (z11 - z01)) / (x1 - x0);
}

double ChFunctionInterp2D::GetDerY(double x, double y) const {
    EnsureGrid();

    if (m_y.empty()) {
        return 0.0;
    }

    const int nx = static_cast<int>(m_x.size());
    const int ny = static_cast<int>(m_y.size());

    if (ny == 1) {
        return 0.0;
    }

    if (nx == 1) {
        const int j = FindCell(m_y, y, m_last_iy);
        return LinearDer(m_y[j], m_z(0, j), m_y[j + 1], m_z(0, j + 1), y, m_extrapolate);
    }

    bool out_x = false;
    bool out_y = false;
    EvalBilinear(x, y, &out_x, &out_y);
    if (!m_extrapolate && out_y) {
        return 0.0;
    }

    const int i = FindCell(m_x, x, m_last_ix);
    const int j = FindCell(m_y, y, m_last_iy);
    const double x0 = m_x[i];
    const double x1 = m_x[i + 1];
    const double y0 = m_y[j];
    const double y1 = m_y[j + 1];
    double tx = (x1 != x0) ? (x - x0) / (x1 - x0) : 0.0;
    if (!m_extrapolate) {
        tx = std::clamp(tx, 0.0, 1.0);
    }

    const double z00 = m_z(i, j);
    const double z10 = m_z(i + 1, j);
    const double z01 = m_z(i, j + 1);
    const double z11 = m_z(i + 1, j + 1);

    return ((1.0 - tx) * (z01 - z00) + tx * (z11 - z10)) / (y1 - y0);
}

double ChFunctionInterp2D::GetStartX() const {
    if (m_table.empty()) {
        return 0.0;
    }
    return m_table.begin()->first;
}

double ChFunctionInterp2D::GetEndX() const {
    if (m_table.empty()) {
        return 0.0;
    }
    return m_table.rbegin()->first;
}

double ChFunctionInterp2D::GetStartY() const {
    bool found = false;
    double y_min = 0.0;
    for (const auto& x_row : m_table) {
        if (x_row.second.empty()) {
            continue;
        }
        const double y = x_row.second.begin()->first;
        if (!found) {
            y_min = y;
            found = true;
        } else {
            y_min = std::min(y_min, y);
        }
    }
    return y_min;
}

double ChFunctionInterp2D::GetEndY() const {
    bool found = false;
    double y_max = 0.0;
    for (const auto& x_row : m_table) {
        if (x_row.second.empty()) {
            continue;
        }
        const double y = x_row.second.rbegin()->first;
        if (!found) {
            y_max = y;
            found = true;
        } else {
            y_max = std::max(y_max, y);
        }
    }
    return y_max;
}

void ChFunctionInterp2D::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChFunctionInterp2D>();
    archive_out << CHNVP(m_table);
    archive_out << CHNVP(m_extrapolate);
}

void ChFunctionInterp2D::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChFunctionInterp2D>();
    archive_in >> CHNVP(m_table);
    archive_in >> CHNVP(m_extrapolate);
    m_grid_dirty = true;
    m_grid_complete = false;
    m_last_ix = 0;
    m_last_iy = 0;
}

}  // end namespace chrono
