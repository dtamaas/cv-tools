#include <iostream>
#include "ocam-functions.h"

struct Settings {
    std::string calibFileName = "../example/undistortion/calib_results_w960_h600.txt";
    std::string inputFileNames = "../example/undistortion/inputs/input";
    std::string resultFileNames = "../example/undistortion/results/result";
    std::string extension = "jpg";
    int lastImageNr = 4;
    float scaleFactor = 4.0;
};

void undistortImages() {
    Settings settings;
    ocam_model o;

    get_ocam_model(&o, settings.calibFileName.c_str());

    int i = 0;
    while (i <= settings.lastImageNr) {
        std::string inputPath = settings.inputFileNames + std::to_string(i) + "." + settings.extension;
        cv::Mat image = cv::imread(inputPath);

        if (image.empty()) {
            std::cout << "Could not read image: " << inputPath << std::endl;
            i++;
            continue;
        }

        cv::Mat result = cv::Mat::zeros(image.size(), image.type());
        
        double point3D[3] = { 100, 200, -300 };
        double point2D[2];

        world2cam(point2D, point3D, &o);
        cam2world(point3D, point2D, &o);
        
        cv::Mat map_x(image.size(), CV_32FC1);
        cv::Mat map_y(image.size(), CV_32FC1);

        create_perspecive_undistortion_LUT(map_x, map_y, &o, settings.scaleFactor);

        cv::remap(image, result, map_x, map_y, cv::INTER_CUBIC, 0);

        std::string result_path = settings.resultFileNames + std::to_string(i) + "." + settings.extension;
        cv::imwrite(result_path, result);
        std::cout << "Processing image: " << inputPath << std::endl;
        i++;
    }
}

void undistortVideo() {
    std::cout << "TODO: implement video undistortion" << std::endl;
}

int main(int argc, char *argv[]) {

    std::cout << "\nThis tool can be used to undistort images/video based on Scaramuzza's omnidirectional camera calibration toolbox.\n" << std::endl;

    int action;
    while (action) {
        std::cout << "Do you want to undistort a set of images or a video?" << std::endl;
        std::cout << "\t[0] Exit" << std::endl;
        std::cout << "\t[1] Set of images" << std::endl;
        std::cout << "\t[2] Video" << std::endl;
        std::cout << ">>" && std::cin >> action;

        switch (action) {
            case 0: break; 
            case 1: undistortImages(); break;
            case 2: undistortVideo(); break;
        }
    }

    return 0;
}
