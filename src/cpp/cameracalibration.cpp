#include <cameracalibration.h>

nlohmann::json cameracalibration::calibrate(const std::string &input_video, float square_width, float marker_width, int board_width, int board_height)
{
    // Aruco Board
    cv::aruco::Dictionary aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board = new cv::aruco::CharucoBoard(cv::Size(board_width, board_height), square_width / 0.0254, marker_width / 0.0254, aruco_dict);
    cv::aruco::CharucoDetector charuco_detector(*charuco_board);

    // Video capture
    cv::VideoCapture video_capture(input_video);
    int frame_count = 0;
    cv::Size frame_shape;

    // Detection output
    std::vector<cv::Point2f> all_corners;
    std::vector<int> all_ids;
    std::vector<int> all_counter;

    while (video_capture.isOpened())
    {
        cv::Mat frame;
        video_capture >> frame;

        if (frame.empty())
        {
            break;
        }

        // Limit FPS
        frame_count++;

        if (frame_count % 10 != 0)
        {
            continue;
        }

        // Detect
        cv::Mat frame_gray;
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        frame_shape = frame_gray.size();

        std::vector<cv::Point2f> charuco_corners;
        std::vector<int> charuco_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<int> marker_ids;

        charuco_detector.detectBoard(frame_gray, charuco_corners, charuco_ids, marker_corners, marker_ids);

        cv::Mat debug_image = frame;

        if (!charuco_ids.empty() && charuco_ids.size() >= 4)
        {
            all_corners.insert(all_corners.end(), charuco_corners.begin(), charuco_corners.end());
            all_ids.insert(all_ids.end(), charuco_ids.begin(), charuco_ids.end());
            all_counter.push_back(charuco_ids.size());

            cv::aruco::drawDetectedMarkers(debug_image, marker_corners, marker_ids);
            cv::aruco::drawDetectedCornersCharuco(debug_image, charuco_corners, charuco_ids);
        }

        cv::imshow("Frame", debug_image);
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    video_capture.release();
    cv::destroyAllWindows();

    // Calibrate
    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> r_vecs, t_vecs;
    std::vector<double> std_dev_intrinsics, std_dev_extrinsics, per_view_errors;

    std::vector<std::vector<cv::Point2f>> corners = {all_corners};
    std::vector<std::vector<int>> ids = {all_ids};

    cv::aruco::calibrateCameraCharuco(
        corners, ids, charuco_board, frame_shape, camera_matrix, dist_coeffs,
        r_vecs, t_vecs, std_dev_intrinsics, std_dev_extrinsics, per_view_errors, cv::CALIB_RATIONAL_MODEL);

    std::cout << "Completed camera calibration" << std::endl;

    // Save calibration output
    nlohmann::json camera_model = {
        {"camera_matrix", std::vector<double>(camera_matrix.begin<double>(), camera_matrix.end<double>())},
        {"distortion_coefficients", std::vector<double>(dist_coeffs.begin<double>(), dist_coeffs.end<double>())},
        {"avg_reprojection_error", cv::mean(per_view_errors)[0]},
        {"num_images", static_cast<int>(all_counter.size())}};

    return camera_model;
}
