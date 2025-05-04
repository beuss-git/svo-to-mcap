#include "zed_camera.hpp"
#include "../utils/zed_utils.hpp"

zed::Status zed::ZEDCamera::init(config::Camera const& camera_cfg)
{
    m_name = camera_cfg.name;

    sl::InitParameters init_parameters {};
    init_parameters.input.setFromSVOFile(camera_cfg.svo_path.string().c_str());
    init_parameters.coordinate_units = camera_cfg.coordinate_units;
    init_parameters.svo_real_time_mode = false;
    init_parameters.depth_mode = camera_cfg.depth_mode;

    std::cout << "Opening file svo file (" << camera_cfg.svo_path.string()
              << ")...\n";

    sl::ERROR_CODE zed_open_state = m_zed.open(init_parameters);
    if (zed_open_state != sl::ERROR_CODE::SUCCESS) {
        return { zed_open_state,
            fmt::format("Camera Open {}.", static_cast<int>(zed_open_state)) };
    }

    auto status = initialize_channel_images(camera_cfg);
    if (!status.ok()) {
        return status;
    }
    return {};
}

zed::Status zed::ZEDCamera::grab(sl::RuntimeParameters rt_parameters)
{
    auto const err = m_zed.grab(rt_parameters);
    if (err != sl::ERROR_CODE::SUCCESS) {
        if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
            m_done = true;
            return { err,
                fmt::format("End of svo file {}.", static_cast<int>(err)) };
        }
        return { err, fmt::format("Camera Grab {}.", static_cast<int>(err)) };
    }
    return {};
}

zed::Status zed::ZEDCamera::initialize_channel_images(
    config::Camera const& camera_cfg)
{
    auto const image_size
        = m_zed.getCameraInformation().camera_configuration.resolution;
    for (auto const& channel : camera_cfg.channels) {
        if (std::holds_alternative<sl::VIEW>(channel.type)) {
            auto view = std::get<sl::VIEW>(channel.type);
            auto mat_type = zed_utils::get_mat_type(view);
            if (view == sl::VIEW::SIDE_BY_SIDE) {
                m_channel_images.push_back(ChannelImage { .name = channel.name,
                    .mat = sl::Mat(
                        sl::Resolution(image_size.width * 2, image_size.height),
                        mat_type),
                    .type = view,
                    .frame_id
                    = zed_utils::get_frame_id(camera_cfg.name, view) });
            } else {
                m_channel_images.push_back(ChannelImage { .name = channel.name,
                    .mat = sl::Mat(image_size, mat_type),
                    .type = view,
                    .frame_id
                    = zed_utils::get_frame_id(camera_cfg.name, view) });
            }

        } else if (std::holds_alternative<sl::MEASURE>(channel.type)) {
            auto measure = std::get<sl::MEASURE>(channel.type);
            auto mat_type = zed_utils::get_mat_type(measure);

            m_channel_images.push_back(ChannelImage { .name = channel.name,
                .mat = sl::Mat(image_size, mat_type),
                .type = measure,
                .frame_id
                = zed_utils::get_frame_id(camera_cfg.name, measure) });
        }
    }
    return {};
}
