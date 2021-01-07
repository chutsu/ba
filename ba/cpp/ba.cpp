#include "ba.hpp"

static mat2_t J_intrinsics(const mat3_t &K) {
  // J = [K[0, 0], 0.0,
  //      0.0, K[1, 1]];
  mat2_t J = zeros(2, 2);
  J(0, 0) = K(0, 0);
  J(1, 1) = K(1, 1);
  return J;
}

static matx_t J_project(const vec3_t &p_C) {
  const real_t x = p_C(0);
  const real_t y = p_C(1);
  const real_t z = p_C(2);

  // J = [1 / z, 0, -x / z^2,
  //      0, 1 / z, -y / z^2];
  matx_t J = zeros(2, 3);
  J(0, 0) = 1.0 / z;
  J(1, 1) = 1.0 / z;
  J(0, 2) = -x / (z * z);
  J(1, 2) = -y / (z * z);
  return J;
}

static mat3_t J_camera_rotation(const quat_t &q_WC,
                                const vec3_t &r_WC,
                                const vec3_t &p_W) {
  const mat3_t C_WC = q_WC.toRotationMatrix();
  const mat3_t C_CW = C_WC.transpose();
  return C_CW * skew(p_W - r_WC);
}

static mat3_t J_camera_translation(const quat_t &q_WC) {
  const mat3_t C_WC = q_WC.toRotationMatrix();
  const mat3_t C_CW = C_WC.transpose();
  return -C_CW;
}

static mat3_t J_target_point(const quat_t &q_WC) {
  const mat3_t C_WC = q_WC.toRotationMatrix();
  const mat3_t C_CW = C_WC.transpose();
  return C_CW;
}

int ba_residual_size(ba_data_t &data) {
  // Calculate residual size
  int r_size = 0;
  for (int k = 0; k < data.nb_frames; k++) {
    r_size += data.point_ids[k][0];
  }
  r_size = r_size * 2;
  // ^ Scale 2 because each pixel error are size 2

  return r_size;
}

vecx_t ba_residuals(ba_data_t &data) {
  // Initialize memory for residuals
  int r_size = ba_residual_size(data);
  vecx_t r{r_size};

  // Loop over time
  int res_idx = 0; // Residual index
  for (int k = 0; k < data.nb_frames; k++) {
    // Form camera pose and its inverse
    const mat4_t T_WC = data.cam_poses[k].T();
    const mat4_t T_CW = T_WC.inverse();

    // Get point ids and measurements at time step k
    const int nb_ids = data.point_ids[k][0];
    const int *point_ids = &data.point_ids[k][1];

    for (int i = 0; i < nb_ids; i++) {
      // Get point in world frame and transform to camera frame
      const int id = point_ids[i];
      const vec3_t p_W{data.points[id]};
      const vec3_t p_C = tf_point(T_CW, p_W);

      // Project point in camera frame down to image plane
      const vec3_t x{p_C(0) / p_C(2), p_C(1) / p_C(2), 1.0};
      const vec2_t z_hat = (data.cam_K * x).head(2);

      // Calculate reprojection error
      const vec2_t z = data.keypoints[k][i];
      r.block(res_idx, 0, 2, 1) = z - z_hat;
      res_idx += 2;
    }
  }

  return r;
}

matx_t ba_jacobian(ba_data_t &data) {
  // Initialize memory for jacobian
  int J_rows = ba_residual_size(data);
  int J_cols = (data.nb_frames * 6) + (data.nb_points * 3);
  matx_t J = zeros(J_rows, J_cols);

  // Loop over camera poses
  int pose_idx = 0;
  int meas_idx = 0;

  for (int k = 0; k < data.nb_frames; k++) {
    // Form camera pose
    const mat4_t T_WC = data.cam_poses[k].T();
    const quat_t q_WC = tf_quat(T_WC);
    const vec3_t r_WC = tf_trans(T_WC);
    const mat4_t T_CW = T_WC.inverse();

    // Get point ids and measurements at time step k
    const int nb_ids = data.point_ids[k][0];
    const int *point_ids = &data.point_ids[k][1];

    // Loop over observations at time k
    for (int i = 0; i < nb_ids; i++) {
      // Get point in world frame and transform to camera frame
      const int id = point_ids[i];
      const vec3_t p_W{data.points[id]};
      const vec3_t p_C = tf_point(T_CW, p_W);

      // Camera pose jacobian
      const int rs = meas_idx * 2;
      int cs = pose_idx * 6;
      const mat2_t J_K = J_intrinsics(data.cam_K);
      const matx_t J_P = J_project(p_C);
      const matx_t J_h = J_K * J_P;
      const mat3_t J_C = J_camera_rotation(q_WC, r_WC, p_W);
      const mat3_t J_r = J_camera_translation(q_WC);
      const matx_t J_cam_rot = -1 * J_h * J_C;
      const matx_t J_cam_pos = -1 * J_h * J_r;
      J.block(rs, cs, 2, 3) = J_cam_rot;
      J.block(rs, cs + 3, 2, 3) = J_cam_pos;

      // Point jacobian
      cs = (data.nb_frames * 6) + point_ids[i] * 3;
      const matx_t J_point = -1 * J_h * J_target_point(q_WC);
      J.block(rs, cs, 2, 3) = J_point;

      meas_idx++;
    }
    pose_idx++;
  }

  return J;
}

void ba_update(ba_data_t &data, const vecx_t &e, const matx_t &E) {
  const real_t lambda = 1e-4;  // LM damping term

  // Solve Gauss-Newton system [H dx = g]: Solve for dx
  matx_t H = E.transpose() * E; // Hessian approx: H = J^t J
  matx_t H_diag = (H.diagonal().asDiagonal());
  H = H + lambda * H_diag; // R. Fletcher trust region mod
  const vecx_t g = -E.transpose() * e;
  const vecx_t dx = H.ldlt().solve(g);   // Cholesky decomp

  // Update camera poses
  for (int k = 0; k < data.nb_frames; k++) {
    const int s = k * 6;

    // Update camera rotation
    const vec3_t dalpha{dx(s), dx(s + 1), dx(s + 2)};
    const quat_t q = data.cam_poses[k].rot();
    const quat_t dq = quat_delta(dalpha);
    data.cam_poses[k].set_rot(dq * q);

    // Update camera position
    const vec3_t r_WC = data.cam_poses[k].trans();
    const vec3_t dr_WC{dx(s + 3), dx(s + 4), dx(s + 5)};
    data.cam_poses[k].set_trans(r_WC + dr_WC);
  }

  // Update points
  for (int i = 0; i < data.nb_points; i++) {
    const int s = (data.nb_frames * 6) + (i * 3);
    const vec3_t dp_W{dx(s), dx(s + 1), dx(s + 2)};
    data.points[i][0] += dp_W(0);
    data.points[i][1] += dp_W(1);
    data.points[i][2] += dp_W(2);
  }
}

real_t ba_cost(const vecx_t &e) {
  return 0.5 * e.transpose() * e;
}

void ba_solve(ba_data_t &data) {
  int max_iter = 10;
  real_t cost_prev = 0.0;

  for (int iter = 0; iter < max_iter; iter++) {
    // Update
    struct timespec t_start = tic();
    const vecx_t e = ba_residuals(data);
    const matx_t E = ba_jacobian(data);
    ba_update(data, e, E);

    // Calculate reprojection error
    double sse = 0.0;
    int N = 0;
    for (int i = 0; i < e.size(); i += 2) {
      const real_t dx = e(i);
      const real_t dy = e(i + 1);
      const real_t reproj_error = sqrt(dx * dx + dy * dy);
      sse += reproj_error * reproj_error;
      N++;
    }
    const double rmse_reproj_error = sqrt(sse / N);

    // Print cost
    const real_t cost = ba_cost(e);
    printf("  - iter[%d]   ", iter);
    printf("cost: %.2e   ", cost);
    printf("time: %.3fs   ", toc(&t_start));
    printf("rmse_reproj_error: %.2fpx\n", rmse_reproj_error);

    // Termination criteria
    real_t cost_diff = fabs(cost - cost_prev);
    if (cost_diff < 1.0e-3) {
      printf("Done!\n");
      break;
    }
    cost_prev = cost;
  }
}

void ba_save(const ba_data_t &data, std::string &save_dir) {
  // Create save directory
  const char last_char = save_dir[save_dir.length() - 1];
  const std::string postfix = (last_char == '/') ? "" : "/";
  save_dir += postfix;
  const std::string cmd = "mkdir -p " + save_dir;
  const int retval = system(cmd.c_str());
  if (retval != 0) {
    printf("Error! Failed to save results to [%s]", save_dir.c_str());
  }

  // Save camera matrix
  {
    const std::string csv_path = save_dir + "camera.csv";
    FILE *camera_csv = fopen(csv_path.c_str(), "w");
    fprintf(camera_csv, "#camera_K\n");
    fprintf(camera_csv, "%f, 0.0000, %f\n", data.cam_K(0, 0), data.cam_K(0, 2));
    fprintf(camera_csv, "0.0000, %f, %f\n", data.cam_K(1, 1), data.cam_K(1, 2));
    fprintf(camera_csv, "0.0000, 0.0000, 1.0000\n");
    fclose(camera_csv);
  }

  // Save camera poses
  {
    const std::string csv_path = save_dir + "camera_poses.csv";
    FILE *poses_csv = fopen(csv_path.c_str(), "w");
    fprintf(poses_csv, "#qw,qx,qy,qz,rx,ry,rz\n");
    for (const auto &pose : data.cam_poses) {
      const auto &q = pose.rot();
      const auto &r = pose.trans();
      fprintf(poses_csv, "%f,%f,%f,%f,", q.w(), q.x(), q.y(), q.z());
      fprintf(poses_csv, "%f,%f,%f", r(0), r(1), r(2));
      fprintf(poses_csv, "\n");
    }
    fclose(poses_csv);
  }

  // Save target pose
  {
    const std::string csv_path = save_dir + "target_pose.csv";
    FILE *target_csv = fopen(csv_path.c_str(), "w");
    fprintf(target_csv, "#qw,qx,qy,qz,rx,ry,rz\n");
    {
      const auto &q = data.target_pose.rot();
      const auto &r = data.target_pose.trans();
      fprintf(target_csv, "%f,%f,%f,%f,", q.w(), q.x(), q.y(), q.z());
      fprintf(target_csv, "%f,%f,%f", r(0), r(1), r(2));
      fprintf(target_csv, "\n");
    }
    fclose(target_csv);
  }

  // Save keypoints
  {
    const std::string csv_path = save_dir + "keypoints.csv";
    FILE *keypoints_csv = fopen(csv_path.c_str(), "w");
    fprintf(keypoints_csv, "#size,keypoints\n");
    for (const auto &kps : data.keypoints) {
      int nb_kps = kps.size();
      fprintf(keypoints_csv, "%d,", nb_kps * 2);
      for (int j = 0; j < nb_kps; j++) {
        fprintf(keypoints_csv, "%f,", kps[j](0));
        fprintf(keypoints_csv, "%f", kps[j](1));
        if (j + 1 < nb_kps) {
          fprintf(keypoints_csv, ",");
        }
      }
      fprintf(keypoints_csv, "\n");
    }
    fclose(keypoints_csv);
  }

  // Save point ids
  {
    const std::string csv_path = save_dir + "point_ids.csv";
    FILE *point_ids_csv = fopen(csv_path.c_str(), "w");
    fprintf(point_ids_csv, "#size,point_ids\n");
    for (int i = 0; i < data.nb_ids; i++) {
      int nb_ids = data.point_ids[i][0];
      fprintf(point_ids_csv, "%d,", data.point_ids[i][0]);
      for (int j = 1; j < (nb_ids + 1); j++) {
        fprintf(point_ids_csv, "%d", data.point_ids[i][j]);
        if (j + 1 < (nb_ids + 1)) {
          fprintf(point_ids_csv, ",");
        }
      }
      fprintf(point_ids_csv, "\n");
    }
    fclose(point_ids_csv);
  }

  // Save points
  {
    const std::string csv_path = save_dir + "points.csv";
    FILE *points_csv = fopen(csv_path.c_str(), "w");
    fprintf(points_csv, "#x,y,z\n");
    for (int i = 0; i < data.nb_points; i++) {
      fprintf(points_csv, "%f,", data.points[i][0]);
      fprintf(points_csv, "%f,", data.points[i][1]);
      fprintf(points_csv, "%f\n", data.points[i][2]);
    }
    fclose(points_csv);
  }
}
