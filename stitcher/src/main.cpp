#include <iostream>
#include <fstream>
#include <math.h>
#include <opencv2/opencv.hpp>

struct Settings {
  // Number of cameras
  int cameraNumber = 5;
  
  // Images to stitch
  std::vector<std::string> inputFileNames = {
    "../example/stitching/inputs/images/stitch1.jpg",
    "../example/stitching/inputs/images/stitch2.jpg",
    "../example/stitching/inputs/images/stitch3.jpg",
    "../example/stitching/inputs/images/stitch4.jpg",
    "../example/stitching/inputs/images/stitch5.jpg",
  };

  std::string outputFileName = "../example/stitching/results/result.jpg";

  int cameraTypes = 2;
  std::vector<std::string> intrinsicFileNames = {
    "../example/stitching/inputs/camera-params/K1.txt",
    "../example/stitching/inputs/camera-params/K2.txt",
  };

  std::vector<std::string> rotationFileNames = {
    "../example/stitching/inputs/camera-params/R1.txt",
    "../example/stitching/inputs/camera-params/R2.txt",
    "../example/stitching/inputs/camera-params/R3.txt",
    "../example/stitching/inputs/camera-params/R4.txt",
    "../example/stitching/inputs/camera-params/R5.txt",
    "../example/stitching/inputs/camera-params/R6.txt",
    "../example/stitching/inputs/camera-params/R7.txt",
    "../example/stitching/inputs/camera-params/R8.txt",
  };

  // std::vector<std::string> translationFileNames = {
  //   "t"
  // };
};

template <typename T>
void read(T &data, const std::string& instruction) {
  std::cout << instruction << std::endl;
  std::cout << ">>" && std::cin >> data;
}

cv::Mat read_parameter(const std::string& path, const int& rows = 3, const int& cols = 3) {
  cv::Mat m = cv::Mat(rows, cols, CV_64F);
  std::ifstream file(path);

  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      std::string str;
      file >> str;
      m.at<double>(i, j) = std::stod(str);
    }
  }

  file.close();
  return m;
}

cv::Point3d transform(const cv::Point3d& point, const cv::Mat& rotation_matrix, const cv::Mat& translation_vector) {
  cv::Point3f transformed_point;
  cv::Mat t_point(3, 1, CV_64F);

  t_point.at<double>(0, 0) = point.x;
  t_point.at<double>(1, 0) = point.y;
  t_point.at<double>(2, 0) = point.z;

  cv::Mat res = rotation_matrix * (t_point + translation_vector);
  return cv::Point3d(res.at<double>(0, 0), res.at<double>(1, 0), res.at<double>(2, 0));
}

void stitch_images() {
  Settings settings;
  std::vector<cv::Mat> images;
  std::vector<cv::Mat> intrinsics;
  std::vector<cv::Mat> r_mats;
  std::vector<double> focal_lengths;
  std::vector<cv::Mat> r_y_mats;
  std::vector<cv::Mat> rotations;

  // Store the image paths in a vector
  for (int i = 0; i < settings.cameraNumber; ++i) {
    cv::Mat img = cv::imread(settings.inputFileNames[i]);
    images.push_back(img);
  }

  // The height and width; it needs to be the same for every image in this case
  int HEIGHT = images[0].size().height;
  int WIDTH = images[0].size().width;

  // Read the INTRINSIC camera parameters for every types of camera; in this case normal + fisheye
  for (int i = 0; i < settings.cameraTypes; ++i) {
    cv::Mat K = read_parameter(settings.intrinsicFileNames[i]);
    intrinsics.push_back(K);
  }

  // Read the EXTRINSIC camera parameters, R and t
  int extrinsicsNumber = (settings.cameraNumber - 1) * 2;
  for (int i = 0; i < extrinsicsNumber; ++i) {
    cv::Mat R = read_parameter(settings.rotationFileNames[i]);
    r_mats.push_back(-R.t());
  }

  // Constants of image centers on the x and y axes
  double c_x = intrinsics[0].at<double>(0,2);
  double c_y = intrinsics[0].at<double>(1,2);

  // Extraction of the focal lengths of the of the camera types
  // Here I took the average of the focal lengths of the x and y axes
  for (int i = 0; i < settings.cameraTypes; ++i) {
    double f = (intrinsics[i].at<double>(0,0) + intrinsics[i].at<double>(1,1)) / 2;
    focal_lengths.push_back(f);
  }

  // Extraction of the rotation around the y axis from the rotations of the cameras
  // We do this for every parameter
  // 
  for (int i = 0; i < r_mats.size(); ++i) {
    // I couldn't find out why I used the asin function here.
    // https://en.wikipedia.org/wiki/Euler_angles
    // https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
    double theta = asin(r_mats[i].at<double>(2,0));
    
    // https://en.wikipedia.org/wiki/Rotation_matrix
    cv::Mat r_y(3, 3, CV_64F);
    r_y.at<double>(0, 0) = cos(theta); r_y.at<double>(0, 1) = 0; r_y.at<double>(0, 2) = sin(theta); 
    r_y.at<double>(1, 0) = 0; r_y.at<double>(1, 1) = 1; r_y.at<double>(1, 2) = 0; 
    r_y.at<double>(2, 0) = -sin(theta); r_y.at<double>(2, 1) = 0; r_y.at<double>(2, 2) = cos(theta);
    r_y_mats.push_back(r_y);
  }

  // TODO: refactor
  // Calculate the rotations cumulatively
  cv::Mat rotation1 = r_y_mats[0];
  cv::Mat rotation2 = r_y_mats[1];
  cv::Mat rotation3 = (r_y_mats[3] * r_y_mats[2].inv()) * rotation2;
  cv::Mat rotation4 = (r_y_mats[5] * r_y_mats[4].inv()) * rotation3;
  cv::Mat rotation5 = (r_y_mats[7] * r_y_mats[6].inv()) * rotation4;
  rotations.push_back(rotation1);
  rotations.push_back(rotation2);
  rotations.push_back(rotation3);
  rotations.push_back(rotation4);
  rotations.push_back(rotation5);

  // Define a zero translation vector; we use this as we don't count the translation between the cameras, only the rotations
  cv::Mat t0(3, 1, CV_64F);
  t0.at<double>(0, 0) = 0;
  t0.at<double>(1, 0) = 0;
  t0.at<double>(2, 0) = 0;

  // The scaling is only good for two camera types
  double f_scale = focal_lengths[0] / focal_lengths[1];
  double s = *max_element(focal_lengths.begin(), focal_lengths.end());

  cv::Mat output_img = cv::Mat::zeros(HEIGHT * 2, WIDTH * 2, images[0].type());
  int tr_x = 0;
  int tr_y = (output_img.rows / 2) - ( (HEIGHT / f_scale) / 2);
  
  // TODO: add image ordering
  // For every camera...
  for (int i = 1; i <= settings.cameraNumber; ++i) {
    double f;

    // Change the focal length for the middle (fisheye) camera
    if (i == 3) {
      f = focal_lengths[1];
    } else {
      f = focal_lengths[0];
    }

    // Iterate through every image point
    for (int y = 0; y < HEIGHT; y++) {
      for (int x = 0; x < WIDTH; x++) {
        // Back projected point & its transformed (rotated) point; from image coodrinate system to world coordinate system
        cv::Point3d bp_point(static_cast<double>(x) - c_x, static_cast<double>(y) - c_y, f);
        cv::Point3d t_point = transform(bp_point, rotations[i - 1], t0);

        // Cylindrical projection
        double h = bp_point.y / sqrt(f * f + bp_point.x * bp_point.x);
        double delta;

        // This handles the overlapping; it is because of the atan2 gives bad result when x and z are both negative
        if (i != 1 && t_point.x < 0 && t_point.z < 0) {
          t_point.x = -t_point.x;
          t_point.z = -t_point.z;
          delta = atan2(t_point.x, t_point.z);
          delta = delta + M_PI;
        } else {
          delta = atan2(t_point.x, t_point.z);
        }

        // Scaling of the points (different focal lengths)
        int new_x = round((c_x + s * delta) / f_scale);
        int new_y = round((c_y + s * h) / f_scale);

        // Translate the point based on its position to be a panoramic image
        i == 1 && x == 0 && y == 0 ? tr_x = -new_x : tr_x = tr_x;
        output_img.at<cv::Vec3b>(new_y + tr_y, new_x + tr_x) = images[i - 1].at<cv::Vec3b>(y, x);
      }
    }
  }

  // Write results
  cv::imwrite(settings.outputFileName, output_img);
}

void stitch_video() {
    // TODO
}

int main(int argc, char** argv) {
  std::cout << "\nThis tool can be used to stitch images from a cameras in a common system.\n" << std::endl;

  int action;
  while (action) {
    std::cout << "Do you want to stitch a set of images or a video?" << std::endl;
    std::cout << "\t[0] Exit" << std::endl;
    std::cout << "\t[1] Set of images" << std::endl;
    std::cout << "\t[2] Video" << std::endl;
    std::cout << ">>" && std::cin >> action;

    switch (action) {
      case 0: break;
      case 1: stitch_images(); break;
      case 2: stitch_video(); break;
    }
  }

  return 0;
}
