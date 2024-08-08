// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "fieldcalibration.h"

class ReprojectionError
{
public:
  ReprojectionError(const std::vector<Eigen::Vector2d> &observed_corners,
                    const std::vector<Eigen::Vector3d> &model_corners,
                    const Eigen::Matrix3d &camera_matrix,
                    const Eigen::VectorXd &distortion_coeffs)
      : observed_corners_(observed_corners),
        model_corners_(model_corners),
        camera_matrix_(camera_matrix),
        distortion_coeffs_(distortion_coeffs) {}

  template <typename T>
  bool operator()(const T *const tag_translation,
                  const T *const tag_quaternion,
                  T *residuals) const
  {

    // Convert translation and quaternion to a transformation matrix
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(tag_translation);
    Eigen::Quaternion<T> q(tag_quaternion[3], tag_quaternion[0],
                           tag_quaternion[1], tag_quaternion[2]);

    // Ensure the quaternion is normalized
    q.normalize();

    Eigen::Matrix<T, 3, 3> R = q.toRotationMatrix();

    std::cout << t << std::endl
              << q << std::endl;

    // Project each model corner into the image plane
    for (size_t i = 0; i < model_corners_.size(); ++i)
    {
      Eigen::Matrix<T, 3, 1> model_corner_T = model_corners_[i].cast<T>();
      Eigen::Matrix<T, 3, 1> world_point = R * model_corner_T + t;

      // Project the 3D point to 2D
      T x = world_point.x() / world_point.z();
      T y = world_point.y() / world_point.z();

      // Apply intrinsic camera matrix
      T fx = T(camera_matrix_(0, 0));
      T fy = T(camera_matrix_(1, 1));
      T cx = T(camera_matrix_(0, 2));
      T cy = T(camera_matrix_(1, 2));

      T predicted_x = fx * x + cx;
      T predicted_y = fy * y + cy;

      // Apply radial distortion correction if needed
      T r2 = x * x + y * y;
      T distortion = T(1.0) + r2 * (distortion_coeffs_(0) + r2 * (distortion_coeffs_(1) + r2 * distortion_coeffs_(4)));

      predicted_x *= distortion;
      predicted_y *= distortion;

      // Compute residuals (difference between observed and predicted points)
      residuals[2 * i] = predicted_x - T(observed_corners_[i].x());
      residuals[2 * i + 1] = predicted_y - T(observed_corners_[i].y());
    }

    return true;
  }

  static ceres::CostFunction *Create(const std::vector<Eigen::Vector2d> &observed_corners,
                                     const std::vector<Eigen::Vector3d> &model_corners,
                                     const Eigen::Matrix3d &camera_matrix,
                                     const Eigen::VectorXd &distortion_coeffs)
  {
    return new ceres::AutoDiffCostFunction<ReprojectionError, ceres::DYNAMIC, 3, 4>(
        new ReprojectionError(observed_corners, model_corners, camera_matrix, distortion_coeffs),
        observed_corners.size() * 2);
  }

private:
  const std::vector<Eigen::Vector2d> observed_corners_;
  const std::vector<Eigen::Vector3d> model_corners_;
  const Eigen::Matrix3d camera_matrix_;
  const Eigen::VectorXd distortion_coeffs_;
};

std::tuple<Eigen::Matrix<double, 3, 3>, Eigen::Matrix<double, 8, 1>>
fieldcalibration::load_camera_model(std::string path)
{
  Eigen::Matrix<double, 3, 3> camera_matrix;
  Eigen::Matrix<double, 8, 1> camera_distortion;

  std::ifstream file(path);

  nlohmann::json json_data;

  try
  {
    json_data = nlohmann::json::parse(file);
  }
  catch (const nlohmann::json::parse_error &e)
  {
    std::cout << e.what() << std::endl;
  }

  bool isCalibdb = json_data.contains("camera");

  if (!isCalibdb)
  {
    for (int i = 0; i < camera_matrix.rows(); i++)
    {
      for (int j = 0; j < camera_matrix.cols(); j++)
      {
        camera_matrix(i, j) =
            json_data["camera_matrix"][(i * camera_matrix.cols()) + j];
      }
    }

    for (int i = 0; i < camera_distortion.rows(); i++)
    {
      for (int j = 0; j < camera_distortion.cols(); j++)
      {
        camera_distortion(i, j) = json_data["distortion_coefficients"]
                                           [(i * camera_distortion.cols()) + j];
      }
    }
  }
  else
  {
    for (int i = 0; i < camera_matrix.rows(); i++)
    {
      for (int j = 0; j < camera_matrix.cols(); j++)
      {
        try
        {
          camera_matrix(i, j) = json_data["camera_matrix"][i][j];
        }
        catch (...)
        {
          camera_matrix(i, j) =
              json_data["camera_matrix"]["data"][(i * camera_matrix.cols()) + j];
        }
      }
    }

    for (int i = 0; i < camera_distortion.rows(); i++)
    {
      for (int j = 0; j < camera_distortion.cols(); j++)
      {
        try
        {
          camera_distortion(i, j) = json_data["distortion_coefficients"]
                                             [(i * camera_distortion.cols()) + j];
        }
        catch (...)
        {
          try
          {
            camera_distortion(i, j) = json_data["distortion_coefficients"]
                                               [(i * camera_distortion.cols()) + j];
          }
          catch (...)
          {
            camera_distortion(i, j) = 0.0;
          }
        }
      }
    }
  }

  return std::make_tuple(camera_matrix, camera_distortion);
}

std::tuple<Eigen::Matrix<double, 3, 3>, Eigen::Matrix<double, 8, 1>>
fieldcalibration::load_camera_model(nlohmann::json json_data)
{

  // Camera matrix
  Eigen::Matrix<double, 3, 3> camera_matrix;

  for (int i = 0; i < camera_matrix.rows(); i++)
  {
    for (int j = 0; j < camera_matrix.cols(); j++)
    {
      camera_matrix(i, j) =
          json_data["camera_matrix"][(i * camera_matrix.cols()) + j];
    }
  }

  // Distortion coefficients
  Eigen::Matrix<double, 8, 1> camera_distortion;

  for (int i = 0; i < camera_distortion.rows(); i++)
  {
    for (int j = 0; j < camera_distortion.cols(); j++)
    {
      camera_distortion(i, j) = json_data["distortion_coefficients"]
                                         [(i * camera_distortion.cols()) + j];
    }
  }

  return std::make_tuple(camera_matrix, camera_distortion);
}

std::map<int, nlohmann::json> fieldcalibration::load_ideal_map(std::string path)
{
  std::ifstream file(path);
  nlohmann::json json_data = nlohmann::json::parse(file);
  std::map<int, nlohmann::json> ideal_map;

  for (const auto &element : json_data["tags"])
  {
    ideal_map[element["ID"]] = element;
  }

  return ideal_map;
}

Eigen::Matrix<double, 4, 4> fieldcalibration::get_tag_transform(
    std::map<int, nlohmann::json> &ideal_map, int tag_id)
{
  Eigen::Matrix<double, 4, 4> transform =
      Eigen::Matrix<double, 4, 4>::Identity();

  transform.block<3, 3>(0, 0) =
      Eigen::Quaternion<double>(
          ideal_map[tag_id]["pose"]["rotation"]["quaternion"]["W"],
          ideal_map[tag_id]["pose"]["rotation"]["quaternion"]["X"],
          ideal_map[tag_id]["pose"]["rotation"]["quaternion"]["Y"],
          ideal_map[tag_id]["pose"]["rotation"]["quaternion"]["Z"])
          .toRotationMatrix();

  transform(0, 3) = ideal_map[tag_id]["pose"]["translation"]["x"];
  transform(1, 3) = ideal_map[tag_id]["pose"]["translation"]["y"];
  transform(2, 3) = ideal_map[tag_id]["pose"]["translation"]["z"];

  return transform;
}

std::vector<Eigen::Vector3d> getModelCorners(double tag_size)
{
  double half_size = tag_size / 2.0;
  std::vector<Eigen::Vector3d> model_corners;
  model_corners.emplace_back(-half_size, -half_size, 0.0);
  model_corners.emplace_back(half_size, -half_size, 0.0);
  model_corners.emplace_back(half_size, half_size, 0.0);
  model_corners.emplace_back(-half_size, half_size, 0.0);
  return model_corners;
}

Eigen::Matrix<double, 4, 4> fieldcalibration::estimate_tag_pose(
    apriltag_detection_t *tag_detection,
    const Eigen::Matrix<double, 3, 3> &camera_matrix,
    const Eigen::Matrix<double, 8, 1> &camera_distortion, double tag_size)
{
  cv::Mat camera_matrix_cv;
  cv::Mat camera_distortion_cv;

  cv::eigen2cv(camera_matrix, camera_matrix_cv);
  cv::eigen2cv(camera_distortion, camera_distortion_cv);

  std::vector<cv::Point2f> points_2d = {
      cv::Point2f(tag_detection->p[0][0], tag_detection->p[0][1]),
      cv::Point2f(tag_detection->p[1][0], tag_detection->p[1][1]),
      cv::Point2f(tag_detection->p[2][0], tag_detection->p[2][1]),
      cv::Point2f(tag_detection->p[3][0], tag_detection->p[3][1])};

  std::vector<cv::Point3f> points_3d_box_base = {
      cv::Point3f(-tag_size / 2.0, tag_size / 2.0, 0.0),
      cv::Point3f(tag_size / 2.0, tag_size / 2.0, 0.0),
      cv::Point3f(tag_size / 2.0, -tag_size / 2.0, 0.0),
      cv::Point3f(-tag_size / 2.0, -tag_size / 2.0, 0.0)};

  std::vector<double> r_vec;
  std::vector<double> t_vec;

  cv::solvePnP(points_3d_box_base, points_2d, camera_matrix_cv,
               camera_distortion_cv, r_vec, t_vec, false,
               cv::SOLVEPNP_SQPNP);

  cv::Mat r_mat;
  cv::Rodrigues(r_vec, r_mat);

  Eigen::Matrix<double, 4, 4> camera_to_tag{
      {r_mat.at<double>(0, 0), r_mat.at<double>(0, 1), r_mat.at<double>(0, 2),
       t_vec.at(0)},
      {r_mat.at<double>(1, 0), r_mat.at<double>(1, 1), r_mat.at<double>(1, 2),
       t_vec.at(1)},
      {r_mat.at<double>(2, 0), r_mat.at<double>(2, 1), r_mat.at<double>(2, 2),
       t_vec.at(2)},
      {0.0, 0.0, 0.0, 1.0}};

  return camera_to_tag;
}

void fieldcalibration::draw_tag_cube(cv::Mat &frame, Eigen::Matrix<double, 4, 4> camera_to_tag,
                                     const Eigen::Matrix<double, 3, 3> &camera_matrix,
                                     const Eigen::Matrix<double, 8, 1> &camera_distortion,
                                     double tag_size)
{
  cv::Mat camera_matrix_cv;
  cv::Mat camera_distortion_cv;

  cv::eigen2cv(camera_matrix, camera_matrix_cv);
  cv::eigen2cv(camera_distortion, camera_distortion_cv);

  std::vector<cv::Point3f> points_3d_box_base = {
      cv::Point3f(-tag_size / 2.0, tag_size / 2.0, 0.0),
      cv::Point3f(tag_size / 2.0, tag_size / 2.0, 0.0),
      cv::Point3f(tag_size / 2.0, -tag_size / 2.0, 0.0),
      cv::Point3f(-tag_size / 2.0, -tag_size / 2.0, 0.0)};

  std::vector<cv::Point3f> points_3d_box_top = {
      cv::Point3f(-tag_size / 2.0, tag_size / 2.0, -tag_size),
      cv::Point3f(tag_size / 2.0, tag_size / 2.0, -tag_size),
      cv::Point3f(tag_size / 2.0, -tag_size / 2.0, -tag_size),
      cv::Point3f(-tag_size / 2.0, -tag_size / 2.0, -tag_size)};

  std::vector<cv::Point2f> points_2d_box_base = {
      cv::Point2f(0.0, 0.0), cv::Point2f(0.0, 0.0), cv::Point2f(0.0, 0.0),
      cv::Point2f(0.0, 0.0)};

  std::vector<cv::Point2f> points_2d_box_top = {
      cv::Point2f(0.0, 0.0), cv::Point2f(0.0, 0.0), cv::Point2f(0.0, 0.0),
      cv::Point2f(0.0, 0.0)};

  Eigen::Matrix<double, 3, 3> r_vec = camera_to_tag.block<3, 3>(0, 0);
  Eigen::Matrix<double, 3, 1> t_vec = camera_to_tag.block<3, 1>(0, 3);

  cv::Mat r_vec_cv;
  cv::Mat t_vec_cv;

  cv::eigen2cv(r_vec, r_vec_cv);
  cv::eigen2cv(t_vec, t_vec_cv);

  cv::projectPoints(points_3d_box_base, r_vec_cv, t_vec_cv, camera_matrix_cv,
                    camera_distortion_cv, points_2d_box_base);
  cv::projectPoints(points_3d_box_top, r_vec_cv, t_vec_cv, camera_matrix_cv,
                    camera_distortion_cv, points_2d_box_top);

  for (int i = 0; i < 4; i++)
  {
    cv::Point2f &point_base = points_2d_box_base.at(i);
    cv::Point2f &point_top = points_2d_box_top.at(i);

    cv::line(frame, point_base, point_top, cv::Scalar(0, 255, 255), 5);

    int i_next = (i + 1) % 4;
    cv::Point2f &point_base_next = points_2d_box_base.at(i_next);
    cv::Point2f &point_top_next = points_2d_box_top.at(i_next);

    cv::line(frame, point_base, point_base_next, cv::Scalar(0, 255, 255), 5);
    cv::line(frame, point_top, point_top_next, cv::Scalar(0, 255, 255), 5);
  }
}

bool fieldcalibration::process_video_file(
    apriltag_detector_t *tag_detector, int detection_fps,
    const Eigen::Matrix<double, 3, 3> &camera_matrix,
    const Eigen::Matrix<double, 8, 1> &camera_distortion, double tag_size,
    const std::string &path,
    std::map<int, Pose, std::less<int>,
             Eigen::aligned_allocator<std::pair<const int, Pose>>> &poses,
    std::vector<Constraint, Eigen::aligned_allocator<Constraint>> &
        constraints)
{
  cv::namedWindow("Processing Frame", cv::WINDOW_NORMAL);
  cv::VideoCapture video_input(path);

  if (!video_input.isOpened())
  {
    std::cout << "Unable to open video " << path << std::endl;
    return false;
  }

  int video_width = video_input.get(cv::CAP_PROP_FRAME_WIDTH);
  int video_height = video_input.get(cv::CAP_PROP_FRAME_HEIGHT);
  int video_fps = video_input.get(cv::CAP_PROP_FPS);

  if (video_fps <= detection_fps)
  {
    video_input.release();
    cv::destroyAllWindows();
    return false;
  }

  cv::Mat frame;
  cv::Mat frame_gray;
  cv::Mat frame_debug;

  int frame_num = 0;

  while (video_input.read(frame))
  {
    if (frame_num++ % (video_fps / detection_fps) != 0)
    {
      continue;
    }

    std::cout << "Processing " << path << " - Frame " << frame_num << std::endl;

    // Convert color frame to grayscale frame
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

    // Clone color frame for debugging
    frame_debug = frame.clone();

    // Detect tags
    image_u8_t tag_image = {frame_gray.cols, frame_gray.rows, frame_gray.cols, frame_gray.data};
    zarray_t *tag_detections = apriltag_detector_detect(tag_detector, &tag_image);

    // Skip this loop if there are no tags detected
    if (zarray_size(tag_detections) == 0)
    {
      std::cout << "No tags detected" << std::endl;
      continue;
    }

    for (int i = 0; i < zarray_size(tag_detections); i++)
    {
      apriltag_detection_t *tag_detection;
      zarray_get(tag_detections, i, &tag_detection);

      std::vector<Eigen::Vector2d> observed_corners;

      // Extract observed 2D corners from detection
      for (int j = 0; j < 4; j++)
      {
        Eigen::Vector2d corner(tag_detection->p[j][0], tag_detection->p[j][1]);
        observed_corners.push_back(corner);
      }

      // Create and store the constraint
      Constraint constraint(tag_detection->id, observed_corners);
      constraints.push_back(constraint);

      // Draw debug cube (optional)
      Eigen::Matrix<double, 4, 4> camera_to_tag = estimate_tag_pose(tag_detection, camera_matrix, camera_distortion, tag_size);
      draw_tag_cube(frame_debug, camera_to_tag, camera_matrix, camera_distortion, tag_size);
    }

    apriltag_detections_destroy(tag_detections);

    // Show debug
    cv::imshow("Processing Frame", frame_debug);
    cv::waitKey(1);
  }

  video_input.release();
  cv::destroyAllWindows();

  return true;
}

int fieldcalibration::calibrate(std::string input_dir_path, std::string output_file_path,
                                std::string camera_model_path, std::string ideal_map_path,
                                int pinned_tag_id, int detection_fps)
{

  // Silence OpenCV logging
  cv::utils::logging::setLogLevel(
      cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);

  // Load camera model
  const auto [camera_matrix, camera_distortion] =
      load_camera_model(camera_model_path);

  // Load ideal field map
  std::map<int, nlohmann::json> ideal_map = load_ideal_map(ideal_map_path);

  // Apriltag detector
  apriltag_detector_t *tag_detector = apriltag_detector_create();
  tag_detector->nthreads = 8;

  apriltag_family_t *tag_family = tag36h11_create();
  apriltag_detector_add_family(tag_detector, tag_family);

  // Find tag poses
  std::map<int, Pose, std::less<int>,
           Eigen::aligned_allocator<std::pair<const int, Pose>>>
      poses;
  std::vector<Constraint, Eigen::aligned_allocator<Constraint>> constraints;

  for (const auto &entry :
       std::filesystem::directory_iterator(input_dir_path))
  {
    if (entry.path().filename().string()[0] == '.')
    {
      continue;
    }

    const std::string path = entry.path().string();

    bool success =
        process_video_file(tag_detector, detection_fps, camera_matrix,
                           camera_distortion, 0.1651, path, poses, constraints);

    if (!success)
    {
      std::cout << "Unable to process video" << std::endl;
      return -1;
    }
  }

  // Build optimization problem
  ceres::Problem problem;
  ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold;

  // Assume a fixed model for tags
  auto model_corners = getModelCorners(0.1651);

  for (const auto &constraint : constraints)
  {
    ceres::CostFunction *cost_function =
        ReprojectionError::Create(constraint.observed_corners, model_corners, camera_matrix, camera_distortion);

    problem.AddResidualBlock(cost_function, nullptr,
                             poses[constraint.id_tag].p.data(), poses[constraint.id_tag].q.coeffs().data());

    problem.SetManifold(poses[constraint.id_tag].q.coeffs().data(), quaternion_manifold);
  }

  // Pin tag
  auto pinned_tag_iter = poses.find(pinned_tag_id);
  if (pinned_tag_iter != poses.end())
  {
    problem.SetParameterBlockConstant(pinned_tag_iter->second.p.data());
    problem.SetParameterBlockConstant(pinned_tag_iter->second.q.coeffs().data());
  }

  // Solve
  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.num_threads = 10;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;

  // Output
  std::map<int, nlohmann::json> observed_map = ideal_map;

  Eigen::Matrix<double, 4, 4> correction_a;
  correction_a << 0, 0, -1, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;

  Eigen::Matrix<double, 4, 4> correction_b;
  correction_b << 0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 1;

  Eigen::Matrix<double, 4, 4> pinned_tag_transform =
      get_tag_transform(ideal_map, pinned_tag_id);

  for (const auto &[tag_id, pose] : poses)
  {
    Eigen::Matrix<double, 4, 4> transform = Eigen::Matrix<double, 4, 4>::Identity();
    transform.block<3, 3>(0, 0) = pose.q.toRotationMatrix();
    transform.block<3, 1>(0, 3) = pose.p;

    Eigen::Matrix<double, 4, 4> corrected_transform =
        pinned_tag_transform * correction_a * transform * correction_b;
    Eigen::Quaternion<double> corrected_transform_q(corrected_transform.block<3, 3>(0, 0));

    observed_map[tag_id]["pose"]["translation"]["x"] = corrected_transform(0, 3);
    observed_map[tag_id]["pose"]["translation"]["y"] = corrected_transform(1, 3);
    observed_map[tag_id]["pose"]["translation"]["z"] = corrected_transform(2, 3);

    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["X"] = corrected_transform_q.x();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["Y"] = corrected_transform_q.y();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["Z"] = corrected_transform_q.z();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["W"] = corrected_transform_q.w();
  }

  nlohmann::json observed_map_json;
  for (const auto &[tag_id, tag_json] : observed_map)
  {
    observed_map_json["tags"].push_back(tag_json);
  }

  observed_map_json["field"] = {{"length", 16.541}, {"width", 8.211}};

  std::ofstream output_file(output_file_path);
  output_file << observed_map_json.dump(4) << std::endl;

  return 0;
}

int fieldcalibration::calibrate(std::string input_dir_path, std::string output_file_path,
                                nlohmann::json camera_model, std::string ideal_map_path,
                                int pinned_tag_id, int detection_fps)
{

  // Silence OpenCV logging
  cv::utils::logging::setLogLevel(
      cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);

  // Load camera model
  const auto [camera_matrix, camera_distortion] =
      load_camera_model(camera_model);

  // Load ideal field map
  std::map<int, nlohmann::json> ideal_map = load_ideal_map(ideal_map_path);

  // Apriltag detector
  apriltag_detector_t *tag_detector = apriltag_detector_create();
  tag_detector->nthreads = 8;

  apriltag_family_t *tag_family = tag36h11_create();
  apriltag_detector_add_family(tag_detector, tag_family);

  // Find tag poses
  std::map<int, Pose, std::less<int>,
           Eigen::aligned_allocator<std::pair<const int, Pose>>>
      poses;
  std::vector<Constraint, Eigen::aligned_allocator<Constraint>> constraints;

  for (const auto &entry :
       std::filesystem::directory_iterator(input_dir_path))
  {
    if (entry.path().filename().string()[0] == '.' ||
        (entry.path().extension().string() != ".mp4" &&
         entry.path().extension().string() != ".mov" &&
         entry.path().extension().string() != ".m4v" &&
         entry.path().extension().string() != ".mkv"))
    {
      continue;
    }

    const std::string path = entry.path().string();

    bool success =
        process_video_file(tag_detector, detection_fps, camera_matrix,
                           camera_distortion, 0.1651, path, poses, constraints);

    if (!success)
    {
      std::cout << "Unable to process video" << std::endl;
      return -1;
    }
  }

  // Build optimization problem
  ceres::Problem problem;
  ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold;

  // Assume a fixed model for tags
  auto model_corners = getModelCorners(0.1651);

  for (const auto &constraint : constraints)
  {
    ceres::CostFunction *cost_function =
        ReprojectionError::Create(constraint.observed_corners, model_corners, camera_matrix, camera_distortion);

    problem.AddResidualBlock(cost_function, nullptr,
                             poses[constraint.id_tag].p.data(), poses[constraint.id_tag].q.coeffs().data());

    problem.SetManifold(poses[constraint.id_tag].q.coeffs().data(), quaternion_manifold);
  }

  // Pin tag
  auto pinned_tag_iter = poses.find(pinned_tag_id);
  if (pinned_tag_iter != poses.end())
  {
    problem.SetParameterBlockConstant(pinned_tag_iter->second.p.data());
    problem.SetParameterBlockConstant(pinned_tag_iter->second.q.coeffs().data());
  }

  // Solve
  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.num_threads = 10;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;

  // Output
  std::map<int, nlohmann::json> observed_map = ideal_map;

  Eigen::Matrix<double, 4, 4> correction_a;
  correction_a << 0, 0, -1, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;

  Eigen::Matrix<double, 4, 4> correction_b;
  correction_b << 0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 1;

  Eigen::Matrix<double, 4, 4> pinned_tag_transform =
      get_tag_transform(ideal_map, pinned_tag_id);

  for (const auto &[tag_id, pose] : poses)
  {
    Eigen::Matrix<double, 4, 4> transform = Eigen::Matrix<double, 4, 4>::Identity();
    transform.block<3, 3>(0, 0) = pose.q.toRotationMatrix();
    transform.block<3, 1>(0, 3) = pose.p;

    Eigen::Matrix<double, 4, 4> corrected_transform =
        pinned_tag_transform * correction_a * transform * correction_b;
    Eigen::Quaternion<double> corrected_transform_q(corrected_transform.block<3, 3>(0, 0));

    observed_map[tag_id]["pose"]["translation"]["x"] = corrected_transform(0, 3);
    observed_map[tag_id]["pose"]["translation"]["y"] = corrected_transform(1, 3);
    observed_map[tag_id]["pose"]["translation"]["z"] = corrected_transform(2, 3);

    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["X"] = corrected_transform_q.x();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["Y"] = corrected_transform_q.y();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["Z"] = corrected_transform_q.z();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["W"] = corrected_transform_q.w();
  }

  nlohmann::json observed_map_json;
  for (const auto &[tag_id, tag_json] : observed_map)
  {
    observed_map_json["tags"].push_back(tag_json);
  }

  observed_map_json["field"] = {{"length", 16.541}, {"width", 8.211}};

  std::ofstream output_file(output_file_path);
  output_file << observed_map_json.dump(4) << std::endl;

  return 0;
}