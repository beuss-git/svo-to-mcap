#pragma once
#include "../config.hpp"
#include "../utils/status.hpp"
#include <cassert>
#include <fmt/format.h>
#include <magic_enum/magic_enum.hpp>
#include <sl/Camera.hpp>

namespace zed {

struct Status : StatusBase<sl::ERROR_CODE, sl::ERROR_CODE::SUCCESS> {
    using StatusBase::StatusBase;
};

static sl::MAT_TYPE get_mat_type(sl::VIEW view)
{
    switch (view) {
    case sl::VIEW::LEFT:
    case sl::VIEW::RIGHT:
    case sl::VIEW::LEFT_UNRECTIFIED:
    case sl::VIEW::RIGHT_UNRECTIFIED:
    case sl::VIEW::SIDE_BY_SIDE:
    case sl::VIEW::DEPTH:
    case sl::VIEW::CONFIDENCE:
    case sl::VIEW::NORMALS:
    case sl::VIEW::DEPTH_RIGHT:
    case sl::VIEW::NORMALS_RIGHT:
        return sl::MAT_TYPE::U8_C4;
    case sl::VIEW::LEFT_GRAY:
    case sl::VIEW::RIGHT_GRAY:
    case sl::VIEW::LEFT_UNRECTIFIED_GRAY:
    case sl::VIEW::RIGHT_UNRECTIFIED_GRAY:
        return sl::MAT_TYPE::U8_C1;
    default:
        std::cerr << "Unknown view type: " << static_cast<int>(view) << "\n";
        std::exit(1);
    }
}

static sl::MAT_TYPE get_mat_type(sl::MEASURE measure)
{
    switch (measure) {
    case sl::MEASURE::DISPARITY:
    case sl::MEASURE::DEPTH:
    case sl::MEASURE::CONFIDENCE:
    case sl::MEASURE::DISPARITY_RIGHT:
    case sl::MEASURE::DEPTH_RIGHT:
        return sl::MAT_TYPE::F32_C1;
    case sl::MEASURE::XYZ:
    case sl::MEASURE::XYZRGBA:
    case sl::MEASURE::XYZBGRA:
    case sl::MEASURE::XYZARGB:
    case sl::MEASURE::XYZABGR:
    case sl::MEASURE::NORMALS:
    case sl::MEASURE::XYZ_RIGHT:
    case sl::MEASURE::XYZRGBA_RIGHT:
    case sl::MEASURE::XYZBGRA_RIGHT:
    case sl::MEASURE::XYZARGB_RIGHT:
    case sl::MEASURE::XYZABGR_RIGHT:
    case sl::MEASURE::NORMALS_RIGHT:
        return sl::MAT_TYPE::F32_C4;
    case sl::MEASURE::DEPTH_U16_MM:
    case sl::MEASURE::DEPTH_U16_MM_RIGHT:
        return sl::MAT_TYPE::U16_C1;
    default:
        std::cerr << "Unknown measure type: " << static_cast<int>(measure)
                  << "\n";
        std::exit(1);
    }
}

static std::string get_frame_id(
    std::string const& camera_name, std::variant<sl::VIEW, sl::MEASURE> type)
{
    std::string name;
    if (std::holds_alternative<sl::VIEW>(type)) {
        name = magic_enum::enum_name(std::get<sl::VIEW>(type));
    } else if (std::holds_alternative<sl::MEASURE>(type)) {
        name = magic_enum::enum_name(std::get<sl::MEASURE>(type));
    } else {
        assert(false);
    }

    // https://github.com/stereolabs/zed-ros-wrapper/blob/3af19a269b0fcdbd43029f85568cfbd42504fde4/zed_nodelets/src/zed_nodelet/src/zed_wrapper_nodelet.cpp#L1303

    if (name.find("RIGHT") != std::string::npos) {
        return camera_name + "_right_camera_optical_frame";
    }
    // By default the left optical frame is used for everything. The left (non
    // optical) frame id is used for things like object detection, but this is
    // not supported.
    return camera_name + "_left_camera_optical_frame";
}

inline bool is_point_cloud(std::variant<sl::VIEW, sl::MEASURE> type)
{
    if (!std::holds_alternative<sl::MEASURE>(type)) {
        return false;
    }

    auto const measure = std::get<sl::MEASURE>(type);
    switch (measure) {
    case sl::MEASURE::XYZ:
    case sl::MEASURE::XYZRGBA:
    case sl::MEASURE::XYZBGRA:
    case sl::MEASURE::XYZARGB:
    case sl::MEASURE::XYZABGR:
    case sl::MEASURE::XYZ_RIGHT:
    case sl::MEASURE::XYZRGBA_RIGHT:
    case sl::MEASURE::XYZBGRA_RIGHT:
    case sl::MEASURE::XYZARGB_RIGHT:
    case sl::MEASURE::XYZABGR_RIGHT:
        return true;
    default:
        return false;
    }
}

struct ChannelImage {
    std::string name;
    sl::Mat image;
    std::variant<sl::VIEW, sl::MEASURE> type;
    std::string frame_id { "fix_me" };
};

class ZEDCamera {
public:
    Status init(config::Camera const& camera_cfg)
    {
        m_name = camera_cfg.name;

        sl::InitParameters init_parameters {};
        init_parameters.input.setFromSVOFile(
            camera_cfg.svo_path.string().c_str());
        init_parameters.coordinate_units = sl::UNIT::METER;
        init_parameters.svo_real_time_mode = false;
        init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;

        std::cout << "Opening file svo file (" << camera_cfg.svo_path.string()
                  << ")...\n";

        sl::ERROR_CODE zed_open_state = m_zed.open(init_parameters);
        if (zed_open_state != sl::ERROR_CODE::SUCCESS) {
            return Status(zed_open_state,
                fmt::format(
                    "Camera Open {}.", static_cast<int>(zed_open_state)));
        }

        auto status = initialize_channel_images(camera_cfg);
        if (!status.ok()) {
            return status;
        }
        return {};
    }

    Status grab_images()
    {
        auto err = m_zed.grab();
        if (err != sl::ERROR_CODE::SUCCESS) {
            return Status(
                err, fmt::format("Camera Grab {}.", static_cast<int>(err)));
        }
        auto const timestamp
            = m_zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getMilliseconds();
        static auto last_timestamp = timestamp;
        std::cout << "Delta: " << timestamp - last_timestamp << "\n";
        std::cout << "Dropped: " << m_zed.getFrameDroppedCount() << "/"
                  << m_zed.getSVOPosition() << "\n";

        last_timestamp = timestamp;

        for (auto& channel_image : m_channel_images) {

            if (std::holds_alternative<sl::VIEW>(channel_image.type)) {
                err = m_zed.retrieveImage(channel_image.image,
                    std::get<sl::VIEW>(channel_image.type));
            } else if (std::holds_alternative<sl::MEASURE>(
                           channel_image.type)) {
                err = m_zed.retrieveMeasure(channel_image.image,
                    std::get<sl::MEASURE>(channel_image.type));
            } else {
                assert(false);
            }

            if (err != sl::ERROR_CODE::SUCCESS) {
                return Status(err,
                    fmt::format("Camera Retrieve {}.", static_cast<int>(err)));
            }
        }
        return {};
    }

    std::vector<ChannelImage> const& channel_images() const
    {
        return m_channel_images;
    }

    sl::Timestamp timestamp(sl::TIME_REFERENCE time_ref)
    {
        return m_zed.getTimestamp(time_ref);
    }

    sl::CameraInformation camera_information()
    {
        return m_zed.getCameraInformation();
    }

    unsigned int num_lost_frames() { return m_zed.getFrameDroppedCount(); }

    float get_fps()
    {
        return m_zed.getCameraInformation().camera_configuration.fps;
    }

    int get_svo_number_of_frames() { return m_zed.getSVONumberOfFrames(); }
    int get_svo_position() { return m_zed.getSVOPosition(); }
    void set_svo_position(int position) { m_zed.setSVOPosition(position); }

    Status grab(sl::RuntimeParameters rt_parameters = sl::RuntimeParameters())
    {
        auto const err = m_zed.grab(rt_parameters);
        if (err != sl::ERROR_CODE::SUCCESS) {
            if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
                m_done = true;
                return Status(err,
                    fmt::format("End of svo file {}.", static_cast<int>(err)));
            }
            return Status(
                err, fmt::format("Camera Grab {}.", static_cast<int>(err)));
        }
        return {};
    }

    bool done() const { return m_done; }

    std::string name() const { return m_name; }
    sl::Camera& zed() { return m_zed; }

    void close() { m_zed.close(); }

private:
    Status initialize_channel_images(config::Camera const& camera_cfg)
    {
        auto const image_size
            = m_zed.getCameraInformation().camera_configuration.resolution;
        for (auto const& channel : camera_cfg.channels) {
            if (std::holds_alternative<sl::VIEW>(channel.type)) {
                auto view = std::get<sl::VIEW>(channel.type);
                auto mat_type = get_mat_type(view);
                if (view == sl::VIEW::SIDE_BY_SIDE) {
                    m_channel_images.push_back(ChannelImage { channel.name,
                        sl::Mat(sl::Resolution(
                                    image_size.width * 2, image_size.height),
                            mat_type),
                        view, get_frame_id(camera_cfg.name, view) });
                } else {
                    std::cout << "Channel name: " << channel.name << "\n";
                    m_channel_images.push_back(ChannelImage { channel.name,
                        sl::Mat(image_size, mat_type), view,
                        get_frame_id(camera_cfg.name, view) });
                }

            } else if (std::holds_alternative<sl::MEASURE>(channel.type)) {
                auto measure = std::get<sl::MEASURE>(channel.type);
                auto mat_type = get_mat_type(measure);

                std::cout << "Channel name: " << channel.name << "\n";
                m_channel_images.push_back(
                    ChannelImage { channel.name, sl::Mat(image_size, mat_type),
                        measure, get_frame_id(camera_cfg.name, measure) });
            }
        }
        return {};
    }

    std::string m_name;
    sl::Camera m_zed;
    std::vector<ChannelImage> m_channel_images;
    bool m_done = false;
};
}
