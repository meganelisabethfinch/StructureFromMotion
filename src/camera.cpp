//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/camera.h>
#include <headers/image.h>
#include <headers/constants.h>
#include <headers/cli_util.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

Camera::Camera(const Image& image) {
    this->K = cv::Matx<double, 3, 3>{
            FOCAL_LENGTH, 0, (double)(image.data.cols / 2),
            0, FOCAL_LENGTH, (double)(image.data.rows / 2),
            0, 0, 1};

    this->distortion = cv::Mat_<double>::zeros(1, 4);
}

Camera::Camera(cv::Mat intrinsic, cv::Mat distCoeffs) {
    this->K = intrinsic;
    this->distortion = distCoeffs;
}

cv::Matx33d Camera::getCameraMatrix() const {
    return K;
};

double Camera::getFocalLength() const {
    return K(0,0);
}

cv::Point2d Camera::getCentre() const {
    return {K(0,2), K(1,2)};
}

cv::Mat_<double> Camera::getDistortion() {
    return distortion;
}

void Camera::setFocalLength(double fx, double fy) {
    K(0,0) = fx;
    K(1,1) = fy;
}

Camera Camera::Create(const Image& image,
                      const std::string& calibrationDir,
                      int boardWidth, int boardHeight,
                      double squareSize) {
    std::vector<cv::String> calibrationFn;
    CLIUtilities::FindImageFilenames(calibrationDir, calibrationFn);

    std::vector<std::vector<cv::Point3d>> objectPoints;
    std::vector<std::vector<cv::Point2d>> imagePoints;
    std::vector<cv::Point2d> corners;

    // boardSize is interior number of corners
    // NOT number of squares in the chessboard
    cv::Size boardSize = cv::Size(boardWidth, boardHeight);
    int boardn = boardWidth * boardHeight;

    for (size_t k = 0; k < calibrationFn.size(); k++) {
        cv::Mat img = cv::imread(calibrationFn.at(k), cv::IMREAD_COLOR);
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        bool found = false;
        found = cv::findChessboardCorners(img, boardSize, corners,
                                          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

        if (found) {
            // Setup termination criteria for the following
            auto crit = cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1);

            // Further refine the corner locations
            cv::cornerSubPix(gray, corners,
                             cv::Size(5,5),
                             cv::Size(-1,-1),
                             crit);

            if (DEFAULT_VISUAL_DEBUG > VisualDebugLevel::SHOW_EXAMPLES) {
                cv::drawChessboardCorners(gray, boardSize, corners, found);
            }
        }

        std::vector<cv::Point3d> obj;
        for (int i = 0; i < boardHeight; i++) {
            for (int j = 0; j < boardWidth; j++) {
                obj.emplace_back((double)j * squareSize, (double)i * squareSize, 0);
            }
        }

        if (found) {
            std::cout << "Found corners for calibration image "
                    << k << " / " << calibrationFn.size() << std::endl;
            imagePoints.push_back(corners);
            objectPoints.push_back(obj);
        }
    }

    // Calibrate the camera!
    cv::Mat K;
    cv::Mat D;
    std::vector<cv::Mat> rvecs, tvecs;

    cv::calibrateCamera(objectPoints, imagePoints,
                        image.data.size(),
                        K, D,
                        rvecs, tvecs);

    // Create calibrated camera
    return {K, D};
}
