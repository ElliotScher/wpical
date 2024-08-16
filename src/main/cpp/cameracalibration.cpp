#include <cameracalibration.h>
#include <iostream>

bool filter(std::vector<cv::Point2f> charuco_corners, std::vector<int> charuco_ids, std::vector<std::vector<cv::Point2f>> marker_corners, std::vector<int> marker_ids, int board_width, int board_height)
{
    if (charuco_ids.empty() || charuco_corners.empty())
    {
        return false;
    }

    if (charuco_corners.size() < 10 || charuco_ids.size() < 10)
    {
        return false;
    }

    for (int i = 0; i < charuco_ids.size(); i++)
    {
        if (charuco_ids.at(i) > (board_width - 1) * (board_height - 1) - 1)
        {
            return false;
        }
    }

    for (int i = 0; i < marker_ids.size(); i++)
    {
        if (marker_ids.at(i) > ((board_width * board_height) / 2) - 1)
        {
            return false;
        }
    }

    return true;
}

int cameracalibration::calibrate(const std::string &input_video, float square_width, float marker_width, int board_width, int board_height, bool show_debug_window)
{
    // Aruco Board
    cv::aruco::Dictionary aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board = new cv::aruco::CharucoBoard(cv::Size(board_width, board_height), square_width * 0.0254, marker_width * 0.0254, aruco_dict);
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

        cv::Mat debug_image = frame;

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

        if (!filter(charuco_corners, charuco_ids, marker_corners, marker_ids, board_width, board_height))
        {
            continue;
        }

        all_corners.insert(all_corners.end(), charuco_corners.begin(), charuco_corners.end());
        all_ids.insert(all_ids.end(), charuco_ids.begin(), charuco_ids.end());
        all_counter.push_back(charuco_ids.size());

        cv::aruco::drawDetectedMarkers(debug_image, marker_corners, marker_ids);
        cv::aruco::drawDetectedCornersCharuco(debug_image, charuco_corners, charuco_ids);

        if (show_debug_window)
        {
            cv::imshow("Frame", debug_image);
        }
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

    try
    {
        cv::aruco::calibrateCameraCharuco(
            corners, ids, charuco_board, frame_shape, camera_matrix, dist_coeffs,
            r_vecs, t_vecs, std_dev_intrinsics, std_dev_extrinsics, per_view_errors, cv::CALIB_RATIONAL_MODEL);
    }
    catch (...)
    {
        std::cout << "calibration failed" << std::endl;
        return 1;
    }

    // Save calibration output
    nlohmann::json camera_model = {
        {"camera_matrix", std::vector<double>(camera_matrix.begin<double>(), camera_matrix.end<double>())},
        {"distortion_coefficients", std::vector<double>(dist_coeffs.begin<double>(), dist_coeffs.end<double>())},
        {"avg_reprojection_error", cv::mean(per_view_errors)[0]}};

    size_t lastSeparatorPos = input_video.find_last_of("/\\");
    std::string output_file_path;

    // If a separator is found, return the substring from the beginning to the last separator
    if (lastSeparatorPos != std::string::npos)
    {
        output_file_path = input_video.substr(0, lastSeparatorPos).append("/camera calibration.json");
    }

    std::ofstream output_file(output_file_path);
    output_file << camera_model.dump(4) << std::endl;

    std::cout << "calibration succeeded" << std::endl;
    return 0;
}

int cameracalibration::calibrate(const std::string &input_video, float square_width, float marker_width, int board_width, int board_height, double imagerWidthPixels, double imagerHeightPixels, bool show_debug_window)
{
    // Aruco Board
    cv::aruco::Dictionary aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board = new cv::aruco::CharucoBoard(cv::Size(board_width, board_height), square_width * 0.0254, marker_width * 0.0254, aruco_dict);
    cv::aruco::CharucoDetector charuco_detector(*charuco_board);

    // Video capture
    cv::VideoCapture video_capture(input_video);
    int frame_count = 0;
    cv::Size frame_shape;

    // Detection output
    std::vector<std::vector<cv::Point2f>> all_corners;
    std::vector<std::vector<int>> all_ids;

    cv::Size boardSize(board_width - 1, board_height - 1);
    cv::Size imagerSize(imagerWidthPixels, imagerHeightPixels);

    std::vector<std::vector<mrcal_point3_t>> all_points;

    while (video_capture.isOpened())
    {
        cv::Mat frame;
        video_capture >> frame;

        if (frame.empty())
        {
            break;
        }

        // Detect
        cv::Mat frame_gray;
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        cv::Mat debug_image = frame;

        frame_shape = frame_gray.size();

        std::vector<cv::Point2f> charuco_corners;
        std::vector<int> charuco_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<int> marker_ids;

        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;

        mrcal_point3_t points[(board_width - 1) * (board_height - 1)];
        bool ids_processed[(board_width - 1) * (board_height - 1)] = {false};

        charuco_detector.detectBoard(frame_gray, charuco_corners, charuco_ids, marker_corners, marker_ids);

        if (!filter(charuco_corners, charuco_ids, marker_corners, marker_ids, board_width, board_height))
        {
            continue;
        }

        all_corners.push_back(charuco_corners);
        all_ids.push_back(charuco_ids);

        cv::aruco::drawDetectedMarkers(debug_image, marker_corners, marker_ids);
        cv::aruco::drawDetectedCornersCharuco(debug_image, charuco_corners, charuco_ids);

        charuco_board->matchImagePoints(charuco_corners, charuco_ids, objectPoints, imagePoints);

        if (show_debug_window)
        {
            cv::imshow("Frame", debug_image);
        }
        if (cv::waitKey(1) == 'q')
        {
            break;
        }

        for (int i = 0; i < charuco_ids.size(); i++)
        {
            int id = charuco_ids.at(i);
            points[id].x = objectPoints.at(i).x;
            points[id].y = objectPoints.at(i).y;
            points[id].z = 1.0f;
            ids_processed[id] = true;
        }

        for (int i = 0; i < (sizeof(points) / sizeof(points[0])); i++)
        {
            if (!ids_processed[i])
            {
                points[i].x = -1.0f;
                points[i].y = -1.0f;
                points[i].z = -1.0f;
            }
        }
        std::vector<mrcal_point3_t> points_vector(points, points + sizeof(points) / sizeof(points[0]));

        all_points.push_back(points_vector);
    }

    video_capture.release();
    cv::destroyAllWindows();

    if (all_points.empty())
    {
        std::cout << "calibration failed" << std::endl;
        return 1;
    }

    std::vector<mrcal_point3_t> observations_board;
    std::vector<mrcal_pose_t> frames_rt_toref;

    for (const auto &points : all_points)
    {
        size_t total_points = board_width * board_height * points.size();
        observations_board.reserve(total_points);
        frames_rt_toref.reserve(points.size());

        auto ret = getSeedPose(points.data(), boardSize, imagerSize, square_width * 0.0254, 1000);

        observations_board.insert(observations_board.end(), points.begin(), points.end());
        frames_rt_toref.push_back(ret);
    }

    auto cal_result = mrcal_main(observations_board, frames_rt_toref, boardSize, square_width * 0.0254, imagerSize, 1000);

    auto &stats = *cal_result;

    // Camera matrix and distortion coefficients
    std::vector<double> camera_matrix = {
        // fx 0 cx
        stats.intrinsics[0],
        0,
        stats.intrinsics[2],
        // 0 fy cy
        0,
        stats.intrinsics[1],
        stats.intrinsics[3],
        // 0 0 1
        0,
        0,
        1};

    std::vector<double> distortion_coefficients = {
        stats.intrinsics[4],
        stats.intrinsics[5],
        stats.intrinsics[6],
        stats.intrinsics[7],
        stats.intrinsics[8],
        stats.intrinsics[9],
        stats.intrinsics[10],
        stats.intrinsics[11],
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0};

    // Save calibration output
    nlohmann::json result = {
        {"camera_matrix", camera_matrix},
        {"distortion_coefficients", distortion_coefficients},
        {"avg_reprojection_error", stats.rms_error}};

    size_t lastSeparatorPos = input_video.find_last_of("/\\");
    std::string output_file_path;

    if (lastSeparatorPos != std::string::npos)
    {
        output_file_path = input_video.substr(0, lastSeparatorPos).append("/camera calibration.json");
    }

    std::ofstream output_file(output_file_path);
    output_file << result.dump(4) << std::endl;

    return 0;
}