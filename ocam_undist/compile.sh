g++ src/main.cpp src/ocam_functions.cpp -o ocam_undist.out \
    -I /usr/local/include/opencv4 \
    -lopencv_core \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_imgcodecs
