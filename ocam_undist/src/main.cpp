#include <iostream>
#include "ocam_functions.h"

struct Settings {
    std::string base_file_name = "Dev0_Image_w960_h600_fn";
    std::string extension = "jpg";
    std::string calib_file_name = "calib_results.txt";
    float scale_factor = 4.0;
    int first_image_index = 3;
    int last_image_index = 216;
};

template <typename T>
void read(T &data, const std::string& instruction) {
    std::cout << instruction << std::endl;
    std::cout << ">>" && std::cin >> data;
}

void undistort_images() {
    Settings settings;
    std::vector<std::string> paths;
    ocam_model o;

    // read<std::string>(settings.base_file_name, "Add the common name of the images without suffixes and numbers. (i.e: 'image1.jpg' -> 'image')");
    // read<std::string>(settings.extension, "Add the extension of the image files. (i.e: 'jpg', 'png', 'bmp')");
    // read<int>(settings.first_image_index, "Add index of the first image. (i.e: 1 if the set starts from 'image1.jpg')");
    // read<int>(settings.last_image_index, "Add index of the last image. (i.e: 3 if the last image of the set is 'image3.jpg')");
    // read<std::string>(settings.calib_file_name, "Add the name of the calibration file. (i.e: 'calib_results.txt')");
    // read<float>(settings.scale_factor, "Add the scale factor for the image undistortion. (zoom, basic value: 4)");

    for (int i = settings.first_image_index; i < settings.last_image_index + 1; i++) {
        paths.push_back(settings.base_file_name + std::to_string(i) + "." + settings.extension);
    }

    get_ocam_model(&o, settings.calib_file_name.c_str());

    int i = 1;
    while(i - 1 < paths.size()) {
        double point3D[3] = { 100 , 200 , -300 };
        double point2D[2];
        world2cam(point2D, point3D, &o);

        cam2world(point3D, point2D, &o);

        std::string source_path = "./images/" + paths[i - 1];
        cv::Mat image = cv::imread(source_path);
        cv::Mat result = cv::Mat::zeros(image.size(), image.type());

        cv::Mat map_x(image.size(), CV_32FC1);
        cv::Mat map_y(image.size(), CV_32FC1);

        create_perspecive_undistortion_LUT(map_x, map_y, &o, settings.scale_factor);

        cv::remap(image, result, map_x, map_y, cv::INTER_CUBIC, 0);

        std::string result_path = "./results/res_" + paths[i - 1];
        cv::imwrite(result_path, result);
        std::cout << "Processing image number " << i++ << std::endl;
    }
}

void undistort_video() {
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
            case 1: undistort_images(); break;
            case 2: undistort_video(); break;
        }
    }

    return 0;
}