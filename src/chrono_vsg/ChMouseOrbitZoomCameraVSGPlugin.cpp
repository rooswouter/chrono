#include "chrono_vsg/ChMouseOrbitZoomCameraVSGPlugin.h"

#include <algorithm>
#include <cmath>

#include <vsg/ui/PointerEvent.h>

namespace chrono {
namespace vsg3d {

namespace {

ChVector3d rotate_vector_rodrigues(const ChVector3d& v, const ChVector3d& axis_unit, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);

    // Rodrigues' rotation formula:
    // v' = v*c + (axis x v)*s + axis*(axis·v)*(1-c)
    const double axis_dot_v = axis_unit ^ v;
    return v * c + (axis_unit % v) * s + axis_unit * (axis_dot_v * (1.0 - c));
}

ChVector3d up_from_vertical_dir(CameraVerticalDir up_dir) {
    switch (up_dir) {
        case CameraVerticalDir::Y:
            return ChVector3d(0.0, 1.0, 0.0);
        case CameraVerticalDir::Z:
        default:
            return ChVector3d(0.0, 0.0, 1.0);
    }
}

double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

}  // namespace

class ChMouseOrbitZoomCameraVSGPlugin::ChMouseOrbitZoomCameraEventHandlerVSG : public ChEventHandlerVSG {
  public:
    ChMouseOrbitZoomCameraEventHandlerVSG(ChVisualSystemVSG& vsys,
                                          CameraVerticalDir up_dir,
                                          double orbit_sensitivity,
                                          double pan_sensitivity,
                                          double zoom_sensitivity,
                                          double zoom_min_distance,
                                          double zoom_max_distance)
        : m_vsys(vsys),
          m_up_dir(up_dir),
          m_up_unit(up_from_vertical_dir(up_dir).GetNormalized()),
          m_orbit_sensitivity(orbit_sensitivity),
          m_pan_sensitivity(pan_sensitivity),
          m_zoom_sensitivity(zoom_sensitivity),
          m_zoom_min_distance(zoom_min_distance),
          m_zoom_max_distance(zoom_max_distance) {}

    void process(vsg::KeyPressEvent& ev) override { m_last_key = static_cast<int>(ev.keyBase); }

    int GetKey() { int tmp = m_last_key; m_last_key = -1; return tmp; }

    void process(vsg::ButtonPressEvent& ev) override {
        m_last_x = ev.x;
        m_last_y = ev.y;

        m_orbiting = (ev.mask & vsg::BUTTON_MASK_1) != 0;
        m_panning = (ev.mask & vsg::BUTTON_MASK_2) != 0;
        m_zooming = (ev.mask & vsg::BUTTON_MASK_3) != 0;
    }

    void process(vsg::ButtonReleaseEvent& ev) override {
        m_last_x = ev.x;
        m_last_y = ev.y;

        // After release, the mask should reflect which buttons remain pressed.
        m_orbiting = (ev.mask & vsg::BUTTON_MASK_1) != 0;
        m_panning = (ev.mask & vsg::BUTTON_MASK_2) != 0;
        m_zooming = (ev.mask & vsg::BUTTON_MASK_3) != 0;
    }

    void process(vsg::MoveEvent& ev) override {
        if (!m_orbiting && !m_zooming && !m_panning)
            return;

        const double dx = static_cast<double>(ev.x - m_last_x);
        const double dy = static_cast<double>(ev.y - m_last_y);
        m_last_x = ev.x;
        m_last_y = ev.y;

        if (m_orbiting) {
            orbit(dx, dy);
        } else if (m_panning) {
            pan(dx, dy);
        } else if (m_zooming) {
            zoom(dy);
        }
    }
    ChVector3d GetOffset() { return m_offset; };

  private:
    void orbit(double dx, double dy) {
        ChVector3d eye = m_vsys.GetCameraPosition();
        ChVector3d target = m_vsys.GetCameraTarget();

        ChVector3d v = eye - target;
        const double radius = v.Length();
        if (radius < 1e-9)
            return;

        const ChVector3d f = v * (1.0 / radius);  // direction from target to eye

        // 1) Yaw around the vertical axis.
        const double yaw_angle = -dx * m_orbit_sensitivity;
        ChVector3d f_yaw = rotate_vector_rodrigues(f, m_up_unit, yaw_angle);

        // 2) Pitch around camera-right axis, clamped to avoid flipping.
        const double pitch_current = std::asin(clamp(f_yaw ^ m_up_unit, -1.0, 1.0));
        // Avoid using `CH_PI_2` to keep includes minimal.
        const double pitch_limit = 1.5707963267948966 - 0.05;  // pi/2 - ~3 degrees
        const double pitch_target = clamp(pitch_current - dy * m_orbit_sensitivity, -pitch_limit, pitch_limit);
        const double d_pitch = pitch_target - pitch_current;

        ChVector3d right = m_up_unit % f_yaw;  // perpendicular to up and view direction
        if (right.Length() < 1e-9)
            return;
        right.Normalize();

        ChVector3d f_new = rotate_vector_rodrigues(f_yaw, right, d_pitch);
        const ChVector3d eye_new = target + f_new * radius;

        m_vsys.SetCameraPosition(eye_new);
    }

    void pan(double dx, double dy) {
        ChVector3d eye = m_vsys.GetCameraPosition();
        ChVector3d target = m_vsys.GetCameraTarget();
        ChVector3d v = eye - target;

        // Pan perpendicular to the look direction, but still aligned with vertical and horizontal
        v.Normalize();
        ChVector3d up = {0.0, 0.0, 1.0};
        ChVector3d hor = Vcross(v, up);
        ChVector3d ver = Vcross(v, hor);
        m_offset += m_pan_sensitivity * dx * hor - m_pan_sensitivity * dy * ver;
    }


    void zoom(double dy) {
        ChVector3d eye = m_vsys.GetCameraPosition();
        ChVector3d target = m_vsys.GetCameraTarget();
        ChVector3d v = eye - target;
        const double radius = v.Length();
        if (radius < 1e-9)
            return;

        const double ratio = std::exp(-dy * m_zoom_sensitivity);
        double radius_new = radius * ratio;
        radius_new = clamp(radius_new, m_zoom_min_distance, m_zoom_max_distance);

        const ChVector3d dir = v * (1.0 / radius);
        const ChVector3d eye_new = target + dir * radius_new;

        m_vsys.SetCameraPosition(eye_new);
    }

    ChVisualSystemVSG& m_vsys;
    CameraVerticalDir m_up_dir;
    ChVector3d m_up_unit;
    ChVector3d m_offset = {0.0, 0.0, 0.0};

    double m_orbit_sensitivity;
    double m_pan_sensitivity;
    double m_zoom_sensitivity;
    double m_zoom_min_distance;
    double m_zoom_max_distance;

    bool m_orbiting = false;
    bool m_panning = false;
    bool m_zooming = false;

    int32_t m_last_x = 0;
    int32_t m_last_y = 0;
    int m_last_key = -1;
};

ChMouseOrbitZoomCameraVSGPlugin::ChMouseOrbitZoomCameraVSGPlugin(CameraVerticalDir up_dir) : m_up_dir(up_dir) {}

int ChMouseOrbitZoomCameraVSGPlugin::GetKey() {
    if (!m_event_handler)
        return -1;
    return m_event_handler->GetKey();
}

void ChMouseOrbitZoomCameraVSGPlugin::OnAttach() {
    auto& vsys = GetVisualSystemVSG();
    m_event_handler = std::make_shared<ChMouseOrbitZoomCameraEventHandlerVSG>(vsys, m_up_dir, m_orbit_sensitivity, m_pan_sensitivity,
                                                                                m_zoom_sensitivity, m_zoom_min_distance,
                                                                                m_zoom_max_distance);
    AddEventHandler(m_event_handler);
}

ChVector3d ChMouseOrbitZoomCameraVSGPlugin::GetOffset() {
    return m_event_handler->GetOffset();
}

}  // namespace vsg3d
}  // namespace chrono

