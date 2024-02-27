#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <string>

int main() {
    // Define the chessboard size
    int chessboardWidth = 7;
    int chessboardHeight = 7;

    // Create vectors to store 3D world points and 2D image points
    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<std::vector<cv::Point2f>> imgPoints;

    // Generate the 3D world points (assuming the chessboard is on the XY plane)
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < chessboardHeight; ++i) {
        for (int j = 0; j < chessboardWidth; ++j) {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    // Open a video capture object (0 corresponds to the default camera)
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the camera." << std::endl;
        return -1;
    }

    cv::Mat frame;
    int key = 0;
    int save_counter = 0;
    int frame_counter = 0;

    while (true) {
        cap >> frame;

        // Increment frame counter
        frame_counter++;

        // Find chessboard corners
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(frame, cv::Size(chessboardWidth, chessboardHeight), corners);

        if (found && frame_counter % 5 == 0) {
            save_counter++;

            // Draw corners on the image
            cv::drawChessboardCorners(frame, cv::Size(chessboardWidth, chessboardHeight), corners, found);

            // Save the 3D world points and 2D image points
            objPoints.push_back(objp);
            imgPoints.push_back(corners);

            // Save the frame
            cv::imwrite("savedata/calibration_result_" + std::to_string(save_counter) + ".jpg", frame);
        }

        key = cv::waitKey(1);
        if (key == 27 || objPoints.size() >= 100)  // Break the loop on ESC key or after collecting enough calibration images
            break;
    }

    // Calibrate the camera
    cv::Mat cameraMatrix, distCoeffs, rvecs, tvecs;
    cv::calibrateCamera(objPoints, imgPoints, frame.size(), cameraMatrix, distCoeffs, rvecs, tvecs);

    // Display the camera matrix (focal lengths fx and fy)
    std::cout << "Focal Lengths (fx, fy): " << cameraMatrix.at<double>(0, 0) << ", " << cameraMatrix.at<double>(1, 1) << std::endl;

    // Release the camera
    cap.release();

    cv::destroyAllWindows();

    return 0;
}
