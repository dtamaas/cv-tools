#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

struct CalibrationSettings {
    std::string inputFileNames = "../example/calibration/inputs/normal";
    std::string cornerFileNames = "../example/calibration/results/corner";
    std::string calibResultFileName = "../example/calibration/calib-normal";
    std::string extension = "jpg";
    int lastImageNr = 5;
    int horizontalCornerNr = 8;
    int verticalCornerNr = 9;
    float squareLength = 30.0;
};

struct FindCornerResults {
    std::vector<std::vector<cv::Point3f> > calibrationObjectPoints;
    std::vector<std::vector<cv::Point2f> > imagePoints;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::Size imageSize;
};

struct CalibrationResults {
    cv::Mat intrinsicMatrix = cv::Mat(3, 3, CV_64F);
    cv::Mat distortionCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rotationVecs;
    std::vector<cv::Mat> translationVecs;
};

void write(const cv::Mat& mat, const std::string& label, const std::string& path) {
    std::ofstream file(path, std::ios::app);
    file << "# " << label << std::endl;
    for (int i = 0; i < mat.rows; i++) {
		for (int j = 0; j < mat.cols; j++) {
			file << mat.at<double>(i, j) << " ";
		}
		file << std::endl;
	}
    file << std::endl;
    file.close();
}

void write_data(const std::string& path, CalibrationResults results) {
    std::ofstream file(path, std::ios::trunc);
	file.close();

    write(results.intrinsicMatrix, "Intrinsic matrix", path);
    write(results.distortionCoeffs, "Distortion coefficients", path);
    
    file.open(path, std::ios::app);
    file << "# Extrinsic parameter number" << std::endl;
    file << results.rotationVecs.size() << std::endl << std::endl;
    file.close();

    std::vector<cv::Mat> rotationMatrices(results.rotationVecs);
    for (int i = 1; i <= results.rotationVecs.size(); ++i) {
        Rodrigues(results.rotationVecs[i-1], rotationMatrices[i-1]);
        write(rotationMatrices[i-1], "Rotation matrix " + std::to_string(i), path);
    }

    for (int i = 1; i <= results.translationVecs.size(); ++i) {
        write(results.translationVecs[i-1], "Translation vector " + std::to_string(i), path);
    }
}

FindCornerResults findChessboardCorners(const CalibrationSettings& settings) {
    FindCornerResults results;

	std::cout << "Finding chessboard corners..." << std::endl;
    int i = 0;
	while (i <= settings.lastImageNr) {
        std::string inputPath = settings.inputFileNames + std::to_string(i) + "." + settings.extension;
		cv::Mat image = cv::imread(inputPath, cv::IMREAD_GRAYSCALE);

         if (image.empty()) {
            std::cout << "Could not read image: " << inputPath << std::endl;
            i++;
            continue;
        }
        
        if (results.imageSize.empty()) {
            results.imageSize = cv::Size(image.cols, image.rows);
        }

        std::vector<cv::Point2f> imageCorners;

		if (cv::findChessboardCorners(image, cv::Size(settings.horizontalCornerNr, settings.verticalCornerNr), imageCorners, cv::CALIB_CB_ADAPTIVE_THRESH)) {
			cv::cornerSubPix(image, imageCorners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            cv::drawChessboardCorners(image, cv::Size(settings.horizontalCornerNr, settings.verticalCornerNr), imageCorners, true);
			
            if (imageCorners.size() == settings.horizontalCornerNr * settings.verticalCornerNr) {
                std::cout << "Corners found on image: " << inputPath << std::endl;

				std::vector<cv::Point2f> tempImgPoints;
				std::vector<cv::Point3f> tempObjPoints;
				
                for (int j = 0; j < imageCorners.size(); ++j) {
					tempImgPoints.push_back(cv::Point2f(imageCorners[j].x, imageCorners[j].y));
					tempObjPoints.push_back(cv::Point3f(j % settings.horizontalCornerNr * settings.squareLength, j / settings.horizontalCornerNr * settings.squareLength, 0));
                }

				results.imagePoints.push_back(tempImgPoints);
				results.calibrationObjectPoints.push_back(tempObjPoints);
				results.corners.push_back(imageCorners);
                
                std::string resultPath = settings.cornerFileNames + std::to_string(i) + "." + settings.extension;
                cv::imwrite(resultPath, image);
			}
		} else {
			std::cout << "Cannot find corners on image: " << inputPath << std::endl;
		}

        i++;
	}

    return results;
}

void calibrate_normal() {
    CalibrationSettings settings;
    CalibrationResults results;
    FindCornerResults corners = findChessboardCorners(settings);

    std::cout << "Calibrating camera..." << std::endl;

    cv::calibrateCamera(
        corners.calibrationObjectPoints,
        corners.imagePoints,
        corners.imageSize,
        results.intrinsicMatrix,
        results.distortionCoeffs,
        results.rotationVecs,
        results.translationVecs,
        cv::CALIB_FIX_PRINCIPAL_POINT
    );

    std::cout << "Calibration completed!" << std::endl;

    write_data(settings.calibResultFileName + ".txt", results);
}

void extract_extrinsics() {
    // std::vector<std::string> paths;
    // for (int i = first_image_index; i < last_image_index + 1; i++) {
    //     paths.push_back(base_file_name + std::to_string(i) + "." + extension);
    // }

    // get_chessboard_corners(paths);
    // std::cout << "Extracting extrinsic parameters from availabla data..." << std::endl;

    // rotation_vecs.clear();
    // translation_vecs.clear();

    // for (int i = 0; i < paths.size(); i++) {
    //     cv::Mat rot_vec;
    //     cv::Mat t_vec;
    //     cv::solvePnP(calibration_object_points[i], image_points[i], intrinsic_matrix, distortion_coeffs, rot_vec, t_vec);
    //     rotation_vecs.push_back(rot_vec);
    //     translation_vecs.push_back(t_vec);
    // }

    // write_data("./cv_extrinsic_results_" + base_file_name + ".txt");
}

int main(int argc, char *argv[]) {
    int action;

    while (action) {
        std::cout << "\nSelect an option below." << std::endl;
        std::cout << "\t[0] Exit" << std::endl;
        std::cout << "\t[1] Normal camera calbration" << std::endl;
        std::cout << "\t[2] Extract extrinsic parameters" << std::endl;
        std::cout << ">>" && std::cin >> action;

        switch (action) {
            case 0: break;
            case 1: calibrate_normal(); break;
            case 2: extract_extrinsics(); break;
        }
    }

    return 0;
}