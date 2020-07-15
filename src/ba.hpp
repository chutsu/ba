#pragma once
#include "util.hpp"

struct ba_data_t {
  mat3_t cam_K;

  poses_t cam_poses;
  pose_t target_pose;
  int nb_frames;

  std::vector<keypoints_t> keypoints;
  int **point_ids;
  int nb_ids;

  real_t **points;
  int nb_points;

  ba_data_t(const std::string &data_path) {
    cam_K = load_camera(data_path);
    cam_poses = load_camera_poses(data_path);
    target_pose = load_target_pose(data_path)[0];
    nb_frames = cam_poses.size();
    keypoints = load_keypoints(data_path);
    point_ids = load_point_ids(data_path, &nb_ids);
    points = load_points(data_path, &nb_points);
  }

  ~ba_data_t() {
    // Point IDs
    for (int i = 0; i < nb_frames; i++) {
      free(point_ids[i]);
    }
    free(point_ids);

    // Points
    for (int i = 0; i < nb_points; i++) {
      free(points[i]);
    }
    free(points);
  }
};

int ba_residual_size(ba_data_t &data);
vecx_t ba_residuals(ba_data_t &data);
matx_t ba_jacobian(ba_data_t &data);
void ba_update(ba_data_t &data, const vecx_t &e, const matx_t &E);
real_t ba_cost(const vecx_t &e);
void ba_solve(ba_data_t &data);
