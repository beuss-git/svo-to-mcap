#include <iostream>

// ZED includes
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

// Sample includes
#include "utils.hpp"
#include <iostream>
#include <mcap/writer.hpp>

void print(std::string msg_prefix,
           sl::ERROR_CODE err_code = sl::ERROR_CODE::SUCCESS,
           std::string msg_suffix = "");

void progress_bar(float ratio, unsigned int w) {
  unsigned int c = ratio * w;
  for (unsigned int x = 0; x < c; x++)
    std::cout << "=";
  for (unsigned int x = c; x < w; x++)
    std::cout << " ";
  std::cout << (int)(ratio * 100) << "% ";
  std::cout << "\r" << std::flush;
}

static bool g_exit_app = false;
int main() {
  const std::string svo_input_path = "/mnt/ingest/bags/temp8/cam_prt.svo2";

  // Create ZED objects
  sl::Camera zed;

  // Specify SVO path parameter
  sl::InitParameters init_parameters;
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
  sl::Resolution image_size =
      zed.getCameraInformation().camera_configuration.resolution;

  sl::Mat left_image(image_size, sl::MAT_TYPE::U8_C4);
  cv::Mat left_image_ocv = util::sl_mat_2_cv_mat(left_image);

  int nb_frames = zed.getSVONumberOfFrames();
  int svo_position = 0;
  zed.setSVOPosition(svo_position);

  while (!g_exit_app) {
    sl::ERROR_CODE err = zed.grab();
    if (err == sl::ERROR_CODE::SUCCESS) {
      svo_position = zed.getSVOPosition();

      // Retrieve SVO images
      zed.retrieveImage(left_image, sl::VIEW::LEFT);
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

void print(std::string msg_prefix, sl::ERROR_CODE err_code,
           std::string msg_suffix) {
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
