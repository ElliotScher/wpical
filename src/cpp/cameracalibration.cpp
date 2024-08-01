#include <cameracalibration.h>
#include <iostream>

nlohmann::json cameracalibration::calibrate(const std::string &input_video, float square_width, float marker_width, int board_width, int board_height)
{
    std::cout << "test print" << std::endl;
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
        {"avg_reprojection_error", cv::mean(per_view_errors)[0]}};

    return camera_model;
}

nlohmann::json cameracalibration::calibrate(const std::string &input_video, float square_width, float marker_width, int board_width, int board_height, double imagerWidthPixels, double imagerHeightPixels, double focal_length_guess)
{
    cv::aruco::Dictionary aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board = new cv::aruco::CharucoBoard(cv::Size(board_width, board_height), square_width / 0.0254, marker_width / 0.0254, aruco_dict);
    cv::aruco::CharucoDetector charuco_detector(*charuco_board);

    cv::VideoCapture video_capture(input_video);
    cv::Size frame_shape;

    cv::Mat objPts;
    cv::Mat outBoardCorners;
    cv::Mat outLevels;
    std::vector<mrcal_point3_t> outPts;

    while (video_capture.isOpened())
    {
        cv::Mat frame;
        video_capture >> frame;
        cv::Mat debug_image = frame;

        if (frame.empty())
        {
            break;
        }

        // Detect
        cv::Mat frame_gray;
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        frame_shape = frame_gray.size();

        cv::Mat objPoints;
        cv::Mat imgPoints;
        cv::Mat detectedCorners;
        cv::Mat detectedIds;

        charuco_detector.detectBoard(frame_gray, detectedCorners, detectedIds);

        std::vector<cv::Mat> detectedCornersList;

        for (int i = 0; i < detectedCorners.total(); i++)
        {
            detectedCornersList.push_back(detectedCorners.row(i));
        }

        auto charucoboard = *charuco_board;
        charucoboard.matchImagePoints(detectedCornersList, detectedIds, objPoints, imgPoints);

        cv::aruco::drawDetectedCornersCharuco(debug_image, detectedCorners, detectedIds);

        imgPoints.copyTo(outBoardCorners);
        objPoints.copyTo(objPts);

        int size = (board_height - 1) * (board_width - 1);

        std::vector<mrcal_point2_t> boardCorners(size);
        std::vector<mrcal_point3_t> objectPoints(size);
        std::vector<float> levels(size);

        for (int i = 0; i < detectedIds.rows; i++)
        {
            for (int j = 0; j < detectedIds.cols; j++)
            {
                int id = detectedIds.at<int>(i, j);
                auto corner = outBoardCorners.at<cv::Point2f>(i, j);
                auto point = objPts.at<cv::Point3f>(i, j);
                boardCorners[id] = mrcal_point2_t{corner.x, corner.y};
                objectPoints[id] = mrcal_point3_t{point.x, point.y, point.z}; // this part is weird. in the photonvision repo it uses the z coordinate but in the mrcal_test it uses the level
                levels[id] = 1.0f;
                std::cout << std::to_string(objectPoints[id].z) << std::endl;
            }
        }

        std::cout << "added detected ids" << std::endl;

        for (int i = 0; i < boardCorners.size(); i++)
        {
            std::cout << "adding board corners" << std::endl;
            if (boardCorners.at(i).x == NULL || boardCorners.at(i).y == NULL)
            {
                objectPoints[i] = mrcal_point3_t{-1.0f, -1.0f, -1.0f};
                levels[i] = -1.0f;
            }
        }

        std::cout << "added board corners" << std::endl;

        outPts = objectPoints;

        cv::imshow("Frame", debug_image);
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    video_capture.release();
    cv::destroyAllWindows();

    std::cout << "completed image processing" << std::endl;

    cv::Size boardSize = cv::Size(board_width - 1, board_height - 1); // corners not squares?
    cv::Size imagerSize = cv::Size(imagerWidthPixels, imagerHeightPixels);

    std::vector<mrcal_point3_t> observations_board;
    std::vector<mrcal_pose_t> frames_rt_toref;

    size_t total_points = boardSize.width * boardSize.height * observations_board.size();
    observations_board.reserve(total_points);
    frames_rt_toref.reserve(observations_board.size());

    std::cout << "created calibration objects" << std::endl;

    for (const auto value : outPts)
    {
        std::cout << "getting seed poses" << std::endl;
        mrcal_pose_t ret = getSeedPose(&value, boardSize, imagerSize, square_width / 0.0254, focal_length_guess);
        observations_board.insert(observations_board.end(), value);
        frames_rt_toref.push_back(ret);
    }

    std::cout << "got seed poses. Calibrating" << std::endl;

    auto calResult = mrcal_main(observations_board, frames_rt_toref, boardSize, square_width / 0.0254, imagerSize, focal_length_guess);

    auto &stats = *calResult;

    std::cout << "calibrated" << std::endl;

    std::vector<double> cameraMatrix = {
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

    std::vector<double> distortionCoefficients = {
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

    double total = 0;
    int count = 0;

    for (int i = 0; i < stats.residuals.size(); i++)
    {
        total += stats.residuals[i];
        count = count + 1;
    }

    auto avg = total / count;

    nlohmann::json result = {// TODO: FIGURE OUT WHICH INTRINSICS CORRESPOND TO CAMERA MATRIX AND DISTORTION COEFFICIENTS
                             {"camera_matrix", cameraMatrix},
                             {"distortion_coefficients", distortionCoefficients},
                             {"avg_reprojection_error", avg}};

    return result;
}