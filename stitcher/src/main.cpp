#include <iostream>
#include <math.h> 
#include <opencv2/opencv.hpp>

#define PI 3.14159265

int camera_number;
std::string base_file_name;
std::string extension = "jpg";
int camera_type_number;
std::string intrinsic_base_name;
std::string r_base_name;
std::string t_base_name;

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
		std::vector<cv::Mat> images;
		std::vector<cv::Mat> intrinsics;
		std::vector<cv::Mat> r_mats;
		std::vector<cv::Mat> t_vecs;

		read<int>(camera_number, "Add the number of cameras in the system.");
		read<std::string>(base_file_name, "Add the common name of the images without suffixes and numbers. (i.e: 'image1.jpg' -> 'image')");
		read<std::string>(extension, "Add the extension of the image files. (i.e: 'jpg', 'png', 'bmp')");
		read<int>(camera_type_number, "How many camera types are being used?");
		read<std::string>(intrinsic_base_name, "Add the common name of the TEXT files containing the intrinsic parameters. (i.e: 'K1.txt, K2.txt -> K')");
		read<std::string>(r_base_name, "Add the common name of the TEXT files containing the rotation matrices. (i.e: 'R1.txt, R2.txt -> R')");
		read<std::string>(t_base_name, "Add the common name of the TEXT files containing the translation vectors. (i.e: 't1.txt, t2.txt -> t')");

		for (int i = 1; i <= camera_number; ++i) {
				cv::Mat img = cv::imread("./images/" + base_file_name + std::to_string(i) + "." + extension);
				images.push_back(img);
		}

		int HEIGHT = images[0].size().height;
		int WIDTH = images[0].size().width;

		for (int i = 1; i <= camera_type_number; ++i) {
				cv::Mat K = read_parameter("./camera_parameters/" + intrinsic_base_name + std::to_string(i) + ".txt");
				intrinsics.push_back(K);
		}

		int extrinsics_number = (camera_number - 1) * 2;
		for (int i = 1; i <= extrinsics_number; ++i) {
				cv::Mat R = read_parameter("./camera_parameters/" + r_base_name + std::to_string(i) + ".txt");
				cv::Mat t = read_parameter("./camera_parameters/" + t_base_name + std::to_string(i) + ".txt", 3, 1);
				r_mats.push_back(-R.t());
				t_vecs.push_back(t);
		}

		std::vector<double> focal_lengths;
		double c_x = intrinsics[0].at<double>(0,2);
		double c_y = intrinsics[0].at<double>(1,2);

		for (int i = 0; i < camera_type_number; ++i) {
				double f = (intrinsics[i].at<double>(0,0) + intrinsics[i].at<double>(1,1)) / 2;
				focal_lengths.push_back(f);
		}

		// extraction of the rotation around the y axis
		std::vector<cv::Mat> r_y_mats;
		std::vector<cv::Mat> rotations;
		for (int i = 0; i < r_mats.size(); ++i) {
				double theta = asin(r_mats[i].at<double>(2,0));
				
				cv::Mat r_y(3, 3, CV_64F);
				r_y.at<double>(0, 0) = cos(theta); r_y.at<double>(0, 1) = 0; r_y.at<double>(0, 2) = sin(theta); 
				r_y.at<double>(1, 0) = 0; r_y.at<double>(1, 1) = 1; r_y.at<double>(1, 2) = 0; 
				r_y.at<double>(2, 0) = -sin(theta); r_y.at<double>(2, 1) = 0; r_y.at<double>(2, 2) = cos(theta);
				r_y_mats.push_back(r_y);
		}
		

		// TODO: refactor
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

		cv::Mat t0(3, 1, CV_64F);
		t0.at<double>(0, 0) = 0;
		t0.at<double>(1, 0) = 0;
		t0.at<double>(2, 0) = 0;

		// the scaling is only good for two camera types.
		double f_scale = focal_lengths[0] / focal_lengths[1];
		double s = *max_element(focal_lengths.begin(), focal_lengths.end());
	
		cv::Mat output_img = cv::Mat::zeros(HEIGHT * 2, WIDTH * 2, images[0].type());
		int tr_x = 0;
		int tr_y = (output_img.rows / 2) - ( (HEIGHT / f_scale) / 2);
		
		// TODO: add image ordering
		for (int i = 1; i <= camera_number; ++i) {
				double f;

				if (i == 3) {
						f = focal_lengths[1];
				} else {
						f = focal_lengths[0];
				}

				for (int y = 0; y < HEIGHT; y++) {
						for (int x = 0; x < WIDTH; x++) {
								cv::Point3d bp_point(static_cast<double>(x) - c_x, static_cast<double>(y) - c_y, f);
								cv::Point3d t_point = transform(bp_point, rotations[i - 1], t0);

								double h = bp_point.y / sqrt(f * f + bp_point.x * bp_point.x);
								double delta;
								if (i != 1 && t_point.x < 0 && t_point.z < 0) {
										t_point.x = -t_point.x;
										t_point.z = -t_point.z;
										delta = atan2(t_point.x, t_point.z);
										delta = delta + PI;
								} else {
										delta = atan2(t_point.x, t_point.z);
								}

								int new_x = round((c_x + s * delta) / f_scale);
								int new_y = round((c_y + s * h) / f_scale);

								i == 1 && x == 0 && y == 0 ? tr_x = -new_x : tr_x = tr_x;
								output_img.at<cv::Vec3b>(new_y + tr_y, new_x + tr_x) = images[i - 1].at<cv::Vec3b>(y, x);
						}
				}
		}

		std::string result_path = "./results/res_" + base_file_name + "." + extension;
		cv::imwrite(result_path, output_img);
}

void stitch_video() {
	// TODO

	//int width, int height, const vector<Mat>& r_mats, const vector<Mat>& t_vecs) {
	
	// VideoCapture capture1("./videos/Dev4-outside.mkv");
	// VideoCapture capture2("./videos/Dev0-outside.mkv");
	// VideoCapture capture3("./videos/Dev2-outside.mkv");
	// VideoCapture capture4("./videos/Dev1-outside.mkv");
	// VideoCapture capture5("./videos/Dev3-outside.mkv");

	// VideoWriter writer("5cam-outside.mkv", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 4, cv::Size(width, height));
	
	// int i = 1;
	// while(true) {
	// 	Mat frame1;
	// 	Mat frame2;
	// 	Mat frame3;
	// 	Mat frame4;
	// 	Mat frame5;
        
	// 	capture1 >> frame1;
	// 	capture2 >> frame2;
	// 	capture3 >> frame3;
	// 	capture4 >> frame4;
	// 	capture5 >> frame5;

	// 	if (frame1.empty() || frame2.empty() || frame3.empty() || frame4.empty() || frame5.empty()) {
  //           break;
  //       }

	// 	cout << "Writing image number " << i++ << "..." << endl;
	// 	Mat result = project_cylindrical(frame1, frame2, frame3, frame4, frame5, r_mats, t_vecs);
	// 	writer.write(result);
	// }

	// capture1.release();
	// capture2.release();
	// capture3.release();
	// capture4.release();
	// capture5.release();
	// writer.release();
}

int main(int argc, char** argv) {

	std::cout << "\n\tThis tool can be used to stitch images from a cameras in a common system.\n" << std::endl;

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