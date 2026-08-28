#pragma once

#include <memory>

#include "chrono/assets/ChVisualSystem.h"

#include "chrono_vsg/ChEventHandlerVSG.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

namespace chrono {
namespace vsg3d {

/// A `ChVisualSystemVSGPlugin` that lets the user orbit the camera (left mouse)
/// and zoom in/out (right mouse).
///
/// Notes:
/// - This plugin disables the default VSG `vsg::Trackball` camera handler to avoid
///   conflicting mouse interactions.
/// - The orbit pivot is the current VSG camera target (center), retrieved on each mouse event.
class CH_VSG_API ChMouseOrbitZoomCameraVSGPlugin : public ChVisualSystemVSGPlugin {
  public:
    explicit ChMouseOrbitZoomCameraVSGPlugin(CameraVerticalDir up_dir = CameraVerticalDir::Z);

    /// Sensitivity in radians per pixel for orbiting.
    void SetOrbitSensitivity(double radians_per_pixel) { m_orbit_sensitivity = radians_per_pixel; }
    void SetPanSensitivity(double exp_per_pixel) { m_pan_sensitivity = exp_per_pixel; }

    /// Sensitivity for zooming: the zoom is computed as `exp(-dy * m_zoom_sensitivity)`.
    void SetZoomSensitivity(double exp_per_pixel) { m_zoom_sensitivity = exp_per_pixel; }

    /// Clamp the camera-eye distance during zoom.
    void SetZoomRange(double min_distance, double max_distance) {
        m_zoom_min_distance = min_distance;
        m_zoom_max_distance = max_distance;
    }

    /// Return the last pressed key (VSG `KeySymbol` as `int`), or -1 if none.
    int GetKey();

    ChVector3d GetOffset();

    virtual bool DisableDefaultCameraTrackball() const override { return true; }

  protected:
    virtual void OnAttach() override;

  private:
    class ChMouseOrbitZoomCameraEventHandlerVSG;

    CameraVerticalDir m_up_dir;
    double m_orbit_sensitivity = 0.005;
    double m_pan_sensitivity = 0.01;
    double m_zoom_sensitivity = 0.01;
    double m_zoom_min_distance = 0.001;
    double m_zoom_max_distance = 1.0e6;
    
    std::shared_ptr<ChMouseOrbitZoomCameraEventHandlerVSG> m_event_handler;
};

}  // namespace vsg3d
}  // namespace chrono

