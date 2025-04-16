#include <iostream>

// ZED includes
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

// MCAP includes
#include <mcap/writer.hpp>

// Sample includes
#include "config.hpp"
#include "ros2/Ros2ImageWriter.hpp"
#include <iostream>

void print(std::string msg_prefix,
    sl::ERROR_CODE err_code = sl::ERROR_CODE::SUCCESS,
    std::string msg_suffix = "");

void progress_bar(float ratio, unsigned int w)
{
    unsigned int c = ratio * w;
    for (unsigned int x = 0; x < c; x++)
        std::cout << "=";
    for (unsigned int x = c; x < w; x++)
        std::cout << " ";
    std::cout << (int)(ratio * 100) << "% ";
    std::cout << "\r" << std::flush;
}

static bool g_exit_app = false;

int main()
{

    Config const cfg = config::parse("config.yaml").value();
    for (auto cam : cfg.cameras) {
        std::cout << "cam.name: " << cam.name << "\n";
    }

    // const std::string svo_input_path = "/mnt/ingest/bags/temp8/cam_prt.svo2";
    // const std::string svo_input_path =
    // "/home/user/bags/watercam3/test_prt.svo";
    std::string const svo_input_path
        = "/home/user/bags/watercam3_compressed/test_prt_compressed.svo";

    // Create ZED objects
    sl::Camera zed;

    // Specify SVO path parameter
    sl::InitParameters init_parameters {};
    init_parameters.input.setFromSVOFile(svo_input_path.c_str());
    init_parameters.coordinate_units = sl::UNIT::MILLIMETER;

    std::cout << "Opening file svo file (" << svo_input_path << ")...\n";

    // Open the camera
    sl::ERROR_CODE zed_open_state = zed.open(init_parameters);
    if (zed_open_state != sl::ERROR_CODE::SUCCESS) {
        print("Camera Open", zed_open_state, "Exit program.");
        return EXIT_FAILURE;
    }
    // Get image size
    sl::Resolution image_size
        = zed.getCameraInformation().camera_configuration.resolution;

    sl::Mat left_image(image_size, sl::MAT_TYPE::U8_C4);

    std::string outputFilename = "output.mcap";
    mcap::McapWriter writer;
    {
        auto options = mcap::McapWriterOptions("");

        options.compression = mcap::Compression::Lz4;
        auto const res = writer.open(outputFilename, options);
        if (!res.ok()) {
            std::cerr << "Failed to open " << outputFilename
                      << " for writing: " << res.message << std::endl;
            return 1;
        }
    }

    Ros2ImageWriter ros2_image_writer(writer, cfg);
    ros2_image_writer.init();

    std::cout << "Created channel and schema\n";

    int nb_frames = zed.getSVONumberOfFrames();
    int svo_position = 0;
    zed.setSVOPosition(svo_position);

    // TODO: Look at these to see how we can do the rest of the image types:
    // https://github.com/stereolabs/zed-ros-wrapper/blob/3af19a269b0fcdbd43029f85568cfbd42504fde4/zed_nodelets/src/zed_nodelet/src/zed_wrapper_nodelet.cpp#L2427
    // We are missing pointcloud, disparity image, depth image, objects, Path,
    // disparity left and right, confidence left and right and so on. Imu,
    uint32_t frame_index = 0;
    while (!g_exit_app) {
        sl::ERROR_CODE err = zed.grab();
        if (err == sl::ERROR_CODE::SUCCESS) {
            svo_position = zed.getSVOPosition();

            // Retrieve SVO images
            zed.retrieveImage(left_image, sl::VIEW::LEFT);
            auto const timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);

            // FIXME: use the correct frame id (doesn't really matter until we
            // get the point clouds or if we want to project the image into 3D
            // space
            if (!ros2_image_writer.write_image(
                    left_image, timestamp, "fix_me")) {
                zed.close();
                return 1;
            }

            std::cout << "Wrote mcap\n";
        }

        // Display progress
        else if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
            print("SVO end has been reached. Exiting now.");
            g_exit_app = true;
        } else {
            print("Grab Error: ", err);
            g_exit_app = true;
        }

        progress_bar((float)(svo_position / (float)nb_frames), 30);
    }

    zed.close();

    return EXIT_SUCCESS;
}

void print(
    std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix)
{
    std::cout << "[Sample]";
    if (err_code != sl::ERROR_CODE::SUCCESS)
        std::cout << "[Error] ";
    else
        std::cout << " ";
    std::cout << msg_prefix << " ";
    if (err_code != sl::ERROR_CODE::SUCCESS) {
        std::cout << " | " << toString(err_code) << " : ";
        std::cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        std::cout << " " << msg_suffix;
    std::cout << std::endl;
}
