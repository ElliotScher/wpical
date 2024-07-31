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
    std::cout << "Inside calibration" << std::endl;

    cv::aruco::Dictionary aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board = new cv::aruco::CharucoBoard(cv::Size(board_width, board_height), square_width / 0.0254, marker_width / 0.0254, aruco_dict);
    cv::aruco::CharucoDetector charuco_detector(*charuco_board);

    std::cout << "Created calibration board objects" << std::endl;

    cv::VideoCapture video_capture(input_video);
    cv::Size frame_shape;

    std::cout << "created video capture objects" << std::endl;

    std::vector<mrcal_point3_t> objPts;
    std::vector<mrcal_point2_t> outBoardCorners;

    std::cout << "created output vectors" << std::endl;

    while (video_capture.isOpened())
    {
        std::cout << "inside video capture loop" << std::endl;

        cv::Mat frame;
        video_capture >> frame;
        cv::Mat debug_image = frame;

        std::cout << "create debug image" << std::endl;

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

        std::cout << "greyscale" << std::endl;

        charuco_detector.detectBoard(frame_gray, detectedCorners, detectedIds);

        std::cout << "detected board" << std::endl;

        std::vector<cv::Mat> detectedCornersList;

        for (int i = 0; i < detectedCorners.total(); i++)
        {
            std::cout << "adding detected corners" << std::endl;
            detectedCornersList.push_back(detectedCorners.row(i));
        }

        std::cout << "added detected corners" << std::endl;

        auto charucoboard = *charuco_board;
        charucoboard.matchImagePoints(detectedCornersList, detectedIds, objPoints, imgPoints);

        std::cout << "matched image points" << std::endl;

        cv::aruco::drawDetectedCornersCharuco(debug_image, detectedCorners, detectedIds);

        std::cout << "drew detcted corners" << std::endl;

        for (int i = 0; i < imgPoints.total(); i++)
        {
            std::cout << "adding board corners" << std::endl;
            cv::Point2f point = imgPoints.at<cv::Point2f>(i);
            outBoardCorners.push_back(mrcal_point2_t{point.x, point.y});
        }

        std::cout << "added board corners" << std::endl;

        for (int i = 0; i < objPoints.total(); i++)
        {
            std::cout << "adding object points" << std::endl;
            cv::Point3f point = objPoints.at<cv::Point3f>(i);
            objPts.push_back(mrcal_point3_t{point.x, point.y, point.z});
        }

        std::cout << "added object points" << std::endl;

        std::vector<mrcal_point2_t> boardCorners;
        std::vector<mrcal_point3_t> objectPoints;
        std::vector<float> levels;

        std::cout << "created mrcal objects" << std::endl;

        for (int i = 0; i < detectedIds.total(); i++)
        {
            std::cout << "adding detected ids" << std::endl;
            boardCorners.push_back(outBoardCorners[i]);
            objectPoints.push_back(objPts[i]);
            levels.push_back(1.0f);
        }

        std::cout << "added detected ids" << std::endl;

        for (int i = 0; i < boardCorners.size(); i++)
        {
            std::cout << "adding board corners" << std::endl;
            if (boardCorners.at(i).x == NULL || boardCorners.at(i).y == NULL)
            {
                boardCorners.push_back(mrcal_point2_t{-1, -1});
                objectPoints.push_back(mrcal_point3_t{-1, -1, -1});
                levels.push_back(-1.0f);
            }
        }

        std::cout << "added board corners" << std::endl;

        cv::imshow("Frame", debug_image);
        if (cv::waitKey(1) == 'q')
        {
            break;
        }

        imgPoints.release();
        objPoints.release();
        detectedCorners.release();
        detectedIds.release();
    }

    video_capture.release();
    cv::destroyAllWindows();

    std::cout << "completed image processing" << std::endl;
    //  may need to use corners not squares for board dimensions

    cv::Size boardSize = cv::Size(board_width, board_height);
    cv::Size imagerSize = cv::Size(imagerWidthPixels, imagerHeightPixels);

    std::vector<mrcal_point3_t> observations_board;
    std::vector<mrcal_pose_t> frames_rt_toref;

    size_t total_points = boardSize.width * boardSize.height * observations_board.size();
    observations_board.reserve(total_points);
    frames_rt_toref.reserve(observations_board.size());

    std::cout << "created calibration objects" << std::endl;

    for (const auto value : objPts)
    {
        std::cout << "getting seed poses" << std::endl;
        mrcal_pose_t ret = getSeedPose(&value, boardSize, imagerSize, square_width, focal_length_guess);
        observations_board.insert(observations_board.end(), value);
        frames_rt_toref.push_back(ret);
    }

    std::cout << "got seed poses. Calibrating" << std::endl;

    auto calResult = mrcal_main(observations_board, frames_rt_toref, boardSize, square_width, imagerSize, focal_length_guess);

    auto &stats = *calResult;

    std::cout << "calibrated" << std::endl;

    nlohmann::json result = {// TODO: FIGURE OUT WHICH INTRINSICS CORRESPOND TO CAMERA MATRIX AND DISTORTION COEFFICIENTS
                             {"camera_matrix", stats.intrinsics},
                             {"distortion_coefficients", stats.residuals},
                             {"avg_reprojection_error", stats.rms_error}};

    return result;
}