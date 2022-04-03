#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>  

std::string base_file_name, extension;
int first_image_index, last_image_index, horizontal_corner_number, vertical_corner_number;
float square_length;
cv::Size image_size;
std::vector<std::vector<cv::Point3f> > calibration_object_points;
std::vector<std::vector<cv::Point2f> > image_points, corners;

cv::Mat intrinsic_matrix = cv::Mat(3, 3, CV_64F);
cv::Mat distortion_coeffs;
std::vector<cv::Mat> rotation_vecs, translation_vecs;

template <typename T>
void read(T &data, const std::string& instruction) {
    std::cout << instruction << std::endl << ">>";
    std::cin >> data;
}

void read_data() {
    read<std::string>(base_file_name, "Add the common name of the calibration images without suffixes and numbers. (i.e: 'image1.jpg' -> 'image')");
    read<std::string>(extension, "Add the extension of the calibration image files. (i.e: 'jpg', 'png', 'bmp')");
    read<int>(first_image_index, "Add index of the first image. (i.e: 1 if the set starts from 'image1.jpg')");
    read<int>(last_image_index, "Add index of the last image. (i.e: 3 if the last image of the set is 'image3.jpg')");
    read<int>(horizontal_corner_number, "Enter the number of inner corners in the horizontal direction.");
    read<int>(vertical_corner_number, "Enter the number of inner corners in the vertical direction.");
    read<float>(square_length, "Enter the length of the chessboard square in millimeter.");
}

void write(const cv::Mat& mat, const std::string& label, const std::string& path) {
    std::ofstream file(path, std::ios::app);
    file << "#" << label << std::endl;
    for (int i = 0; i < mat.rows; i++) {
		for (int j = 0; j < mat.cols; j++) {
			file << mat.at<double>(i, j) << " ";
		}
		file << std::endl;
	}
    file << std::endl;
    file.close();
}

void write_data(const std::string& path) {
    std::ofstream file(path, std::ios::trunc);
	file.close();

    write(intrinsic_matrix, "Intrinsic matrix", path);
    write(distortion_coeffs, "Distortion coefficients", path);
    
    file.open(path, std::ios::app);
    file << "#Extrinsic parameter number" << std::endl;
    file << rotation_vecs.size() << std::endl << std::endl;
    file.close();

    std::vector<cv::Mat> rotation_matrices(rotation_vecs);
    for (int i = 0; i < rotation_vecs.size(); i++) {
        Rodrigues(rotation_vecs[i], rotation_matrices[i]);
        write(rotation_matrices[i], "Rotation matrix_" + std::to_string(i), path);
    }

    for (int i = 0; i < translation_vecs.size(); i++) {
        write(translation_vecs[i], "Translation vectors_" + std::to_string(i), path);
    }
}

void get_chessboard_corners(const std::vector<std::string>& paths) {
	image_points.clear();
	calibration_object_points.clear();
	std::vector<cv::Point2f> image_corners;

	std::cout << "Finding and drawing chessboard corners..." << std::endl;
    int count = 0;
	for (int i = 0; i < paths.size(); i++) {
		std::cout << paths[i] << ": ";
		cv::Mat image = cv::imread("./images/" + paths[i]);
		image_size = cv::Size(image.cols, image.rows);
		cv::Mat gray;
		cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
		
		bool found_corners = cv::findChessboardCorners(gray, cv::Size(horizontal_corner_number, vertical_corner_number), image_corners, cv::CALIB_CB_ADAPTIVE_THRESH);
		if (found_corners) {
            ++count;
            std::cout << "corner found" << std::endl;
			cv::cornerSubPix(gray, image_corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            cv::drawChessboardCorners(image, cv::Size(horizontal_corner_number, vertical_corner_number), image_corners, found_corners);
			
            if (image_corners.size() == horizontal_corner_number * vertical_corner_number) {

				std::vector<cv::Point2f> temp_img_points;
				std::vector<cv::Point3f> temp_obj_points;
				
                for (int j = 0; j < image_corners.size(); ++j) {
					cv::Point2f img_pt(image_corners[j].x, image_corners[j].y);
					cv::Point3f obj_pt(j % horizontal_corner_number * square_length, j / horizontal_corner_number * square_length, 0);

					temp_img_points.push_back(img_pt);
					temp_obj_points.push_back(obj_pt);
				}
				image_points.push_back(temp_img_points);
				calibration_object_points.push_back(temp_obj_points);
				corners.push_back(image_corners);

                cv::imwrite("./results/res_" + paths[i], image);
			}
		}
		else {
			std::cout << "cannot find corners!" << std::endl;
		}
	}
    std::cout << "chessboards found: " << count << std::endl;
}

void calibrate_normal() {
    read_data();

    std::vector<std::string> paths;
    for (int i = first_image_index; i < last_image_index + 1; i++) {
        paths.push_back(base_file_name + std::to_string(i) + "." + extension);
    }

    get_chessboard_corners(paths);

    std::cout << "Calibrating camera with standard lens..." << std::endl;
    distortion_coeffs = cv::Mat::zeros(8, 1, CV_64F);
    cv::calibrateCamera(calibration_object_points, image_points, image_size, intrinsic_matrix, distortion_coeffs, rotation_vecs, translation_vecs, cv::CALIB_FIX_PRINCIPAL_POINT);
    std::cout << "Calibration completed!" << std::endl;
    write_data("./cv_calib_results_" + base_file_name + ".txt");
}

void calibrate_fisheye() {
    read_data();

    std::vector<std::string> paths;
    for (int i = first_image_index; i < last_image_index + 1; i++) {
        paths.push_back(base_file_name + std::to_string(i) + "." + extension);
    }

    get_chessboard_corners(paths);

    std::cout << "Calibrating camera with fish-eye lens..." << std::endl;
    distortion_coeffs = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat _rvecs, _tvecs;
    cv::fisheye::calibrate(calibration_object_points, image_points, image_size, intrinsic_matrix, distortion_coeffs, _rvecs, _tvecs, cv::fisheye::CALIB_FIX_PRINCIPAL_POINT);

    rotation_vecs.reserve(_rvecs.rows);
    translation_vecs.reserve(_tvecs.rows);
    for (int i = 0; i < _rvecs.rows; i++) {
        cv::Mat _tmp_r = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat _tmp_t = cv::Mat::zeros(3, 1, CV_64F);

        _tmp_r.at<double>(0, 0) = _rvecs.at<double>(i, 0);
        _tmp_r.at<double>(1, 0) = _rvecs.at<double>(i, 1);
        _tmp_r.at<double>(2, 0) = _rvecs.at<double>(i, 2);

        _tmp_t.at<double>(0, 0) = _tvecs.at<double>(i, 0);
        _tmp_t.at<double>(1, 0) = _tvecs.at<double>(i, 1);
        _tmp_t.at<double>(2, 0) = _tvecs.at<double>(i, 2);

        rotation_vecs.push_back(_tmp_r);
        translation_vecs.push_back(_tmp_t);
    }

    std::cout << intrinsic_matrix << std::endl;
    std::cout << std::endl;
    std::cout << distortion_coeffs << std::endl;
}

void extract_extrinsics() {
    read_data();

    std::vector<std::string> paths;
    for (int i = first_image_index; i < last_image_index + 1; i++) {
        paths.push_back(base_file_name + std::to_string(i) + "." + extension);
    }

    get_chessboard_corners(paths);
    std::cout << "Extracting extrinsic parameters from availabla data..." << std::endl;

    rotation_vecs.clear();
    translation_vecs.clear();

    for (int i = 0; i < paths.size(); i++) {
        cv::Mat rot_vec;
        cv::Mat t_vec;
        cv::solvePnP(calibration_object_points[i], image_points[i], intrinsic_matrix, distortion_coeffs, rot_vec, t_vec);
        rotation_vecs.push_back(rot_vec);
        translation_vecs.push_back(t_vec);
    }

    write_data("./cv_extrinsic_results_" + base_file_name + ".txt");
}

void calibrate_stereo() {
    
    std::cout << "You have to calibrate the two camera first!" << std::endl;
    std::cout << "---- Calibrating camera 1 ----" << std::endl;
    calibrate_normal();
    cv::Mat K1(intrinsic_matrix);
    cv::Mat D1(distortion_coeffs);

    std::cout << "---- Calibrating camera 2 ----" << std::endl;
    calibrate_normal();
    cv::Mat K2(intrinsic_matrix);
    cv::Mat D2(distortion_coeffs);

    std::cout << "---- Camera 1 images ----" << std::endl;
    read_data();
    std::vector<std::string> paths;
    for (int i = first_image_index; i < last_image_index + 1; i++) {
        paths.push_back(base_file_name + std::to_string(i) + "." + extension);
    }
    get_chessboard_corners(paths);
    std::vector<std::vector<cv::Point2f> > image_points1 = image_points;
    paths.clear();

    std::cout << "---- Camera 2 images ----" << std::endl;
    read_data();
    for (int i = first_image_index; i < last_image_index + 1; i++) {
        paths.push_back(base_file_name + std::to_string(i) + "." + extension);
    }
    get_chessboard_corners(paths);
    std::vector<std::vector<cv::Point2f> > image_points2 = image_points;

    cv::Mat E, F, R, T, err;
    cv::stereoCalibrate(
        calibration_object_points, 
        image_points1, 
        image_points2, 
        K1, 
        D1, 
        K2, 
        D2, 
        image_size, 
        R, 
        T,
        E,
        F,
        err,
        cv::CALIB_FIX_INTRINSIC);

    std::cout << T << std::endl;
    std::cout << R << std::endl;
    
    rotation_vecs.clear();
    translation_vecs.clear();
    cv::Mat r_vec;
    Rodrigues(R, r_vec);
    rotation_vecs.push_back(r_vec);
    translation_vecs.push_back(T);

    write_data("./cv_stereo_results_" + base_file_name + ".txt");
}


int main(int argc, char *argv[]) {

    std::cout << "\n\tThis tool can be used for camera calibration with OpenCV.\n" << std::endl;

    int action;
     while (action) {
        std::cout << "Select an option below." << std::endl;
        std::cout << "\t[0] Exit" << std::endl;
        std::cout << "\t[1] Normal camera calbration" << std::endl;
        std::cout << "\t[2] Fisheye camera calibration" << std::endl;
        std::cout << "\t[3] Stereo camera calibration" << std::endl;
        std::cout << "\t[4] Extract extrinsic parameters (only usable after recent calibration)" << std::endl;
        read<int>(action, "");

        switch (action) {
            case 0: break; 
            case 1: calibrate_normal(); break;
            case 2: calibrate_fisheye(); break;
            case 3: calibrate_stereo(); break;
            case 4: extract_extrinsics(); break;
        }
    }

    return 0;
}