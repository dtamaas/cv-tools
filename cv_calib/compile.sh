g++ src/main.cpp -o cv_calib.out \
    -I /usr/local/include/opencv4 \
    -lopencv_core \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_imgcodecs \
    -lopencv_calib3d
