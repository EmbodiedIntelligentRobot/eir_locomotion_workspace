#ifndef EHR_MUJOCO_DEPTH_CAMERA_H_
#define EHR_MUJOCO_DEPTH_CAMERA_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include <mujoco/mujoco.h>

namespace MJ_BRIDGE {

// This is the simulator-side representation of the already processed depth
// observation.  It deliberately has no ROS dependency; ehr_bridge copies it
// into prime_msgs::msg::DepthStack on a non-real-time thread.
struct DepthStack {
    static constexpr std::size_t kFrameCount = 8;
    static constexpr std::size_t kHeight = 18;
    static constexpr std::size_t kWidth = 32;
    static constexpr std::size_t kFrameSize = kHeight * kWidth;
    static constexpr std::size_t kDataSize = kFrameCount * kFrameSize;

    std::array<float, kDataSize> data{};
    std::uint32_t source_sequence{0};
    std::uint64_t timestamp_ns{0}; // Simulation time.
    std::uint64_t captured_monotonic_ns{0}; // Acquisition start, steady_clock; never refreshed on read.
    bool valid{false};
};

struct DepthCameraConfig {
    std::string preprocessing_version{"depth_v1"};
    int raw_width{64};
    int raw_height{40};
    int output_width{static_cast<int>(DepthStack::kWidth)};
    int output_height{static_cast<int>(DepthStack::kHeight)};
    int history_length{37};
    int output_frames{static_cast<int>(DepthStack::kFrameCount)};
    int history_skip_frames{5};
    int capture_frequency_hz{50};

    // Provisional ray-distance blind zone; calibrate against the real camera.
    double min_distance{0.28};
    double max_distance{2.5};
    // parkour_depth_v2 native axial-Z sensor range, before Gaussian.
    // Out-of-range/no-hit samples are zero; policy normalization stays 2.5 m.
    double sensor_min_distance{0.25};
    double sensor_max_distance{2.5};
    double horizontal_fov_deg{90.0};
    double vertical_fov_deg{65.0};

    int crop_up{22};
    int crop_down{0};
    int crop_left{16};
    int crop_right{16};

    // The site is the training-frame anchor.  Older robot XMLs contain the
    // Gemini site instead; bind() handles that explicit compatibility alias.
    std::string camera_site_name{"depth_camera"};
};

/**
 * Pure MuJoCo depth camera used by the simulation backend.
 *
 * Physical nearest-surface occlusion with a policy-compatible image layout:
 * pinhole rays, distance-to-image-plane projection, 3x3 Gaussian blur,
 * normalization to [0, 1], and a 37-frame history sampled at policy rate.
 * Groups 0/1/2 include terrain and robot visuals; collision proxies are excluded.
 * Blind-zone hits remain occlusions, never exposing a surface behind them.
 */
class DepthCamera {
public:
    explicit DepthCamera(const DepthCameraConfig& config = DepthCameraConfig{});

    bool bind(const mjModel* model, std::string* error = nullptr);
    bool isBound() const noexcept { return site_id_ >= 0; }
    int siteId() const noexcept { return site_id_; }

    // Captures one processed stack.  model/data must be the MuJoCo physics
    // thread's model/data pair and must already have current derived values.
    bool capture(const mjModel* model, const mjData* data,
                 std::uint32_t source_sequence,
                 std::uint64_t timestamp_ns,
                 DepthStack& output);

    const DepthCameraConfig& config() const noexcept { return config_; }
    void reset();

private:
    static constexpr std::size_t kMaxHistory = 37;
    static constexpr std::size_t kMaxOutputFrames = DepthStack::kFrameCount;

    bool captureFrame(const mjModel* model, const mjData* data,
                      std::array<float, DepthStack::kFrameSize>& frame) const;
    bool captureParkourFrame(const mjModel*, const mjData*,
        std::array<float, DepthStack::kFrameSize>&) const;
    static void gaussianBlur3x3(
        const std::array<float, DepthStack::kFrameSize>& input,
        std::array<float, DepthStack::kFrameSize>& output);
    static double clampFinite(double value, double fallback) noexcept;

    DepthCameraConfig config_;
    int site_id_{-1};
    int housing_body_id_{-1};
    std::array<std::array<float, DepthStack::kFrameSize>, kMaxHistory> history_{};
    std::size_t history_size_{0};
    std::size_t history_next_{0};
};

}  // namespace MJ_BRIDGE

#endif  // EHR_MUJOCO_DEPTH_CAMERA_H_
