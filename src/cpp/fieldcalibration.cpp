// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "fieldcalibration.h"
#include <gtsam/inference/Symbol.h>
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;
#include <gtsam/slam/expressions.h>

class PoseGraphError
{
public:
  PoseGraphError(Pose t_ab_observed)
      : m_t_ab_observed(std::move(t_ab_observed)) {}

  template <typename T>
  bool operator()(const T *const p_a_ptr, const T *const q_a_ptr,
                  const T *const p_b_ptr, const T *const q_b_ptr,
                  T *residuals_ptr) const
  {
    // Tag A
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

    // Tag B
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

    // Rotation between tag A to tag B
    Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

    // Displacement between tag A and tag B in tag A's frame
    Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

    // Error between the orientations
    Eigen::Quaternion<T> delta_q =
        m_t_ab_observed.q.template cast<T>() * q_ab_estimated.conjugate();

    // Residuals
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) =
        p_ab_estimated - m_t_ab_observed.p.template cast<T>();
    residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

    return true;
  }

  static ceres::CostFunction *Create(const Pose &t_ab_observed)
  {
    return new ceres::AutoDiffCostFunction<PoseGraphError, 6, 3, 4, 3, 4>(
        new PoseGraphError(t_ab_observed));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const Pose m_t_ab_observed;
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
    image_u8_t tag_image = {frame_gray.cols, frame_gray.rows, frame_gray.cols,
                            frame_gray.data};
    zarray_t *tag_detections =
        apriltag_detector_detect(tag_detector, &tag_image);

    // Skip this loop if there are no tags detected
    if (zarray_size(tag_detections) == 0)
    {
      std::cout << "No tags detected" << std::endl;
      continue;
    }

    // Find detection with the smallest tag ID
    apriltag_detection_t *tag_detection_min = nullptr;
    zarray_get(tag_detections, 0, &tag_detection_min);

    for (int i = 0; i < zarray_size(tag_detections); i++)
    {
      apriltag_detection_t *tag_detection_i;
      zarray_get(tag_detections, i, &tag_detection_i);

      if (tag_detection_i->id < tag_detection_min->id)
      {
        tag_detection_min = tag_detection_i;
      }
    }

    Eigen::Matrix<double, 4, 4> camera_to_tag_min = estimate_tag_pose(
        tag_detection_min, camera_matrix, camera_distortion, tag_size);

    // Find transformation from smallest tag ID
    for (int i = 0; i < zarray_size(tag_detections); i++)
    {
      apriltag_detection_t *tag_detection_i;
      zarray_get(tag_detections, i, &tag_detection_i);

      // Add tag ID to poses
      if (poses.find(tag_detection_i->id) == poses.end())
      {
        poses[tag_detection_i->id] = {};
        poses[tag_detection_i->id].p = Eigen::Vector3d(0.0, 0.0, 0.0);
        poses[tag_detection_i->id].q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      }

      // Estimate camera to tag pose
      Eigen::Matrix<double, 4, 4> caamera_to_tag = estimate_tag_pose(
          tag_detection_i, camera_matrix, camera_distortion, tag_size);

      // Draw debug cube
      draw_tag_cube(frame_debug, caamera_to_tag, camera_matrix,
                    camera_distortion, tag_size);

      // Skip finding transformation from smallest tag ID to itself
      if (tag_detection_i->id == tag_detection_min->id)
      {
        continue;
      }

      Eigen::Matrix<double, 4, 4> tag_min_to_tag =
          camera_to_tag_min.inverse() * caamera_to_tag;

      // Constraint
      Constraint constraint;
      constraint.id_begin = tag_detection_min->id;
      constraint.id_end = tag_detection_i->id;
      constraint.t_begin_end.p = tag_min_to_tag.block<3, 1>(0, 3);
      constraint.t_begin_end.q =
          Eigen::Quaterniond(tag_min_to_tag.block<3, 3>(0, 0));

      constraints.push_back(constraint);
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

  for (const auto &constraint : constraints)
  {
    auto pose_begin_iter = poses.find(constraint.id_begin);
    auto pose_end_iter = poses.find(constraint.id_end);

    ceres::CostFunction *cost_function =
        PoseGraphError::Create(constraint.t_begin_end);

    problem.AddResidualBlock(cost_function, nullptr,
                             pose_begin_iter->second.p.data(),
                             pose_begin_iter->second.q.coeffs().data(),
                             pose_end_iter->second.p.data(),
                             pose_end_iter->second.q.coeffs().data());

    problem.SetManifold(pose_begin_iter->second.q.coeffs().data(),
                        quaternion_manifold);
    problem.SetManifold(pose_end_iter->second.q.coeffs().data(),
                        quaternion_manifold);
  }

  // Pin tag
  auto pinned_tag_iter = poses.find(pinned_tag_id);
  if (pinned_tag_iter != poses.end())
  {
    problem.SetParameterBlockConstant(pinned_tag_iter->second.p.data());
    problem.SetParameterBlockConstant(
        pinned_tag_iter->second.q.coeffs().data());
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
    // Transformation from pinned tag
    Eigen::Matrix<double, 4, 4> transform =
        Eigen::Matrix<double, 4, 4>::Identity();

    transform.block<3, 3>(0, 0) = pose.q.toRotationMatrix();
    transform.block<3, 1>(0, 3) = pose.p;

    // Transformation from world
    Eigen::Matrix<double, 4, 4> corrected_transform =
        pinned_tag_transform * correction_a * transform * correction_b;
    Eigen::Quaternion<double> corrected_transform_q(
        corrected_transform.block<3, 3>(0, 0));

    observed_map[tag_id]["pose"]["translation"]["x"] =
        corrected_transform(0, 3);
    observed_map[tag_id]["pose"]["translation"]["y"] =
        corrected_transform(1, 3);
    observed_map[tag_id]["pose"]["translation"]["z"] =
        corrected_transform(2, 3);

    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["X"] =
        corrected_transform_q.x();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["Y"] =
        corrected_transform_q.y();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["Z"] =
        corrected_transform_q.z();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["W"] =
        corrected_transform_q.w();
  }

  nlohmann::json observed_map_json;

  for (const auto &[tag_id, tag_json] : observed_map)
  {
    observed_map_json["tags"].push_back(tag_json);
  }

  observed_map_json["field"] = {
      {"length", 16.541},
      {"width", 8.211}};

  // for (const auto& constraint : constraints)
  // {
  // nlohmann::json output_constraint;

  // output_constraint["from_id"] = constraint.id_begin;
  // output_constraint["to_id"] = constraint.id_end;

  // output_constraint["transform"]["translation"]["x"] =
  // constraint.t_begin_end.p[0];
  // output_constraint["transform"]["translation"]["y"] =
  // constraint.t_begin_end.p[1];
  // output_constraint["transform"]["translation"]["z"] =
  // constraint.t_begin_end.p[2];

  // output_constraint["transform"]["rotation"]["x"] =
  // constraint.t_begin_end.q.x();
  // output_constraint["transform"]["rotation"]["y"] =
  // constraint.t_begin_end.q.y();
  // output_constraint["transform"]["rotation"]["z"] =
  // constraint.t_begin_end.q.z();
  // output_constraint["transform"]["rotation"]["w"] =
  // constraint.t_begin_end.q.w();

  // output["constraints"].push_back(output_constraint);

  // std::cout << constraint.id_begin << " -- " << constraint.id_end <<
  // std::endl;
  // }

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

  for (const auto &constraint : constraints)
  {
    auto pose_begin_iter = poses.find(constraint.id_begin);
    auto pose_end_iter = poses.find(constraint.id_end);

    ceres::CostFunction *cost_function =
        PoseGraphError::Create(constraint.t_begin_end);

    problem.AddResidualBlock(cost_function, nullptr,
                             pose_begin_iter->second.p.data(),
                             pose_begin_iter->second.q.coeffs().data(),
                             pose_end_iter->second.p.data(),
                             pose_end_iter->second.q.coeffs().data());

    problem.SetManifold(pose_begin_iter->second.q.coeffs().data(),
                        quaternion_manifold);
    problem.SetManifold(pose_end_iter->second.q.coeffs().data(),
                        quaternion_manifold);
  }

  // Pin tag
  auto pinned_tag_iter = poses.find(pinned_tag_id);
  if (pinned_tag_iter != poses.end())
  {
    problem.SetParameterBlockConstant(pinned_tag_iter->second.p.data());
    problem.SetParameterBlockConstant(
        pinned_tag_iter->second.q.coeffs().data());
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
    // Transformation from pinned tag
    Eigen::Matrix<double, 4, 4> transform =
        Eigen::Matrix<double, 4, 4>::Identity();

    transform.block<3, 3>(0, 0) = pose.q.toRotationMatrix();
    transform.block<3, 1>(0, 3) = pose.p;

    // Transformation from world
    Eigen::Matrix<double, 4, 4> corrected_transform =
        pinned_tag_transform * correction_a * transform * correction_b;
    Eigen::Quaternion<double> corrected_transform_q(
        corrected_transform.block<3, 3>(0, 0));

    observed_map[tag_id]["pose"]["translation"]["x"] =
        corrected_transform(0, 3);
    observed_map[tag_id]["pose"]["translation"]["y"] =
        corrected_transform(1, 3);
    observed_map[tag_id]["pose"]["translation"]["z"] =
        corrected_transform(2, 3);

    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["X"] =
        corrected_transform_q.x();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["Y"] =
        corrected_transform_q.y();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["Z"] =
        corrected_transform_q.z();
    observed_map[tag_id]["pose"]["rotation"]["quaternion"]["W"] =
        corrected_transform_q.w();
  }

  nlohmann::json observed_map_json;

  for (const auto &[tag_id, tag_json] : observed_map)
  {
    observed_map_json["tags"].push_back(tag_json);
  }

  observed_map_json["field"] = {
      {"length", 16.541},
      {"width", 8.211}};

  // for (const auto& constraint : constraints)
  // {
  // nlohmann::json output_constraint;

  // output_constraint["from_id"] = constraint.id_begin;
  // output_constraint["to_id"] = constraint.id_end;

  // output_constraint["transform"]["translation"]["x"] =
  // constraint.t_begin_end.p[0];
  // output_constraint["transform"]["translation"]["y"] =
  // constraint.t_begin_end.p[1];
  // output_constraint["transform"]["translation"]["z"] =
  // constraint.t_begin_end.p[2];

  // output_constraint["transform"]["rotation"]["x"] =
  // constraint.t_begin_end.q.x();
  // output_constraint["transform"]["rotation"]["y"] =
  // constraint.t_begin_end.q.y();
  // output_constraint["transform"]["rotation"]["z"] =
  // constraint.t_begin_end.q.z();
  // output_constraint["transform"]["rotation"]["w"] =
  // constraint.t_begin_end.q.w();

  // output["constraints"].push_back(output_constraint);

  // std::cout << constraint.id_begin << " -- " << constraint.id_end <<
  // std::endl;
  // }

  std::ofstream output_file(output_file_path);
  output_file << observed_map_json.dump(4) << std::endl;

  return 0;
}

gtsam::Pose3 estimateObservationPose(nlohmann::json camera_model, std::vector<TagDetection> tags, std::string ideal_map_path)
{
  std::ifstream file(ideal_map_path);
  nlohmann::json ideal_map = nlohmann::json::parse(file);

  auto fx = camera_model["camera_matrix"][0];
  auto fy = camera_model["camera_matrix"][4];
  auto cx = camera_model["camera_matrix"][2];
  auto cy = camera_model["camera_matrix"][5];

  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  std::vector<double> distortion_coeff_vector = camera_model["distortion coefficients"].get<std::vector<double>>();
  cv::Mat distortion_coeffs = cv::Mat(distortion_coeff_vector);

  std::vector<cv::Point3f> world_points;
  std::vector<cv::Point2f> image_points;

  for (auto &tag : tags)
  {
    for (int i = 0; i < tag.corners.size(); i++)
    {
      image_points.push_back(cv::Point2f(tag.corners[i].first, tag.corners[i].second));
      std::vector<gtsam::Point3> corners = tagmodel::WorldToCorners(tag.id, ideal_map).value();
      world_points.push_back(cv::Point3f(corners[i].x(), corners[i].y(), corners[i].z())); // make sure the corner order given to the function matches up with the corners gotten from worldtocorners()
    }
  }

  if (world_points.empty() || image_points.empty())
  {
    return {};
  }

  cv::Mat rvec, tvec;

  bool success = cv::solvePnP(world_points, image_points, camera_matrix, distortion_coeffs, rvec, tvec);
  if (!success)
  {
    return gtsam::Pose3{gtsam::Rot3{0.0, 0.0, 0.0, 0.0}, gtsam::Point3{0.0, 0.0, 0.0}};
  }

  cv::Mat R;
  cv::Rodrigues(rvec, R);

  Eigen::Matrix3d rotation;
  cv::cv2eigen(R, rotation);
  Eigen::Vector3d translation;
  cv::cv2eigen(tvec, translation);

  return gtsam::Pose3(gtsam::Rot3(rotation), translation);
}

int gtsam_calibrate(std::string input_dir_path, std::string output_file_path,
                    nlohmann::json camera_model, std::string ideal_map_path,
                    int pinned_tag_id, int detection_fps)
{
  auto fx = camera_model["camera_matrix"][0];
  auto fy = camera_model["camera_matrix"][4];
  auto cx = camera_model["camera_matrix"][2];
  auto cy = camera_model["camera_matrix"][5];

  // Camera calibration parameters. Order is [fx fy skew cx cy] in pixels
  gtsam::Cal3_S2 K(fx, fy, 0.0, cx, cy);
  gtsam::noiseModel::Isotropic::shared_ptr cameraNoise =
      gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // Noise on our pose prior. order is rx, ry, rz, tx, ty, tz, and units are
  // [rad] and [m]
  gtsam::Vector6 sigmas;
  sigmas << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3);
  auto posePriorNoise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  // Our platform and camera are coincident
  gtsam::Pose3 robotTcamera{};

  // constant Expression for the calibration we can reuse
  gtsam::Cal3_S2_ cameraCal(K);

  // List of tag observations - TODO: overload processvideofile() to return this?
  std::map<gtsam::Key, std::vector<TagDetection>> points{};

  std::ifstream file(ideal_map_path);
  auto tagLayoutGuess = nlohmann::json::parse(file);

  gtsam::ExpressionFactorGraph graph;

  // Add all our tag observations
  for (const auto &[stateKey, tags] : points)
  {
    for (const TagDetection &tag : tags)
    {
      auto worldPcorners = tagmodel::WorldToCornersFactor(L(tag.id));

      // add each tag corner
      constexpr int NUM_CORNERS = 4;
      for (size_t i = 0; i < NUM_CORNERS; i++)
      {
        // Decision variable - where our camera is in the world
        const gtsam::Pose3_ worldTbody_fac(stateKey);

        // Where we'd predict the i'th corner of the tag to be
        const auto prediction = gtsamutils::PredictLandmarkImageLocationFactor(
            worldTbody_fac, robotTcamera, cameraCal, worldPcorners[i]);

        // where we saw the i'th corner in the image
        gtsam::Point2 measurement = {tag.corners[i].first, tag.corners[i].second};

        // Add this prediction/measurement pair to our graph
        graph.addExpressionFactor(prediction, measurement, cameraNoise);
      }
    }
  }

  auto worldTtag1 = tagmodel::worldToTag(pinned_tag_id, tagLayoutGuess);
  if (!worldTtag1)
  {
    std::cout << "Couldnt find tag in map!" << std::endl;
  }
  graph.addPrior(L(pinned_tag_id), *worldTtag1, posePriorNoise);

  // Initial guess for our optimizer. Needs to be in the right ballpark, but
  // accuracy doesn't super matter
  gtsam::Values initial;

  // Guess for all camera poses
  for (const auto &[stateKey, tags] : points)
  {
    // Initial guess at camera pose based on regular old multi tag pose
    // esitmation
    auto worldTcam_guess = estimateObservationPose(camera_model, tags, tagLayoutGuess);
    initial.insert<gtsam::Pose3>(stateKey, worldTcam_guess);
  }

  for (auto &tag : tagLayoutGuess["tags"].items())
  {
    double tagXPos = tag.value()["pose"]["translation"]["x"];
    double tagYPos = tag.value()["pose"]["translation"]["y"];
    double tagZPos = tag.value()["pose"]["translation"]["z"];
    double tagWQuat = tag.value()["pose"]["rotation"]["quaternion"]["W"];
    double tagXQuat = tag.value()["pose"]["rotation"]["quaternion"]["X"];
    double tagYQuat = tag.value()["pose"]["rotation"]["quaternion"]["Y"];
    double tagZQuat = tag.value()["pose"]["rotation"]["quaternion"]["Z"];

    gtsam::Rot3 rotation(tagWQuat, tagXQuat, tagYQuat, tagZQuat);
    gtsam::Point3 translation(tagXPos, tagYPos, tagZPos);

    initial.insert(L(tag.value()), gtsam::Pose3{rotation, translation});
  }

  /* Optimize the graph and print results */
  gtsam::Values result = gtsam::DoglegOptimizer(graph, initial).optimize();
  std::cout << "final error = " << graph.error(result) << std::endl;

  return 0;
}