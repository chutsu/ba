#include "ba.hpp"

void print_usage() {
  printf("Usage: ba <data>\n");
  printf("Example: ba ./data\n");
}

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
      const mat3_t J_C = J_camera_rotation(q_WC, r_WC, p_W);
      const mat3_t J_r = J_camera_translation(q_WC);
      const matx_t J_cam_rot = -1 * J_K * J_P * J_C;
      const matx_t J_cam_pos = -1 * J_K * J_P * J_r;
      J.block(rs, cs, 2, 3) = J_cam_rot;
      J.block(rs, cs + 3, 2, 3) = J_cam_pos;

      // Point jacobian
      cs = (data.nb_frames * 6) + point_ids[i] * 3;
      const matx_t J_point = -1 * J_K * J_P * J_target_point(q_WC);
      J.block(rs, cs, 2, 3) = J_point;

      meas_idx++;
    }
    pose_idx++;
  }

  return J;
}

void ba_update(ba_data_t &data, const vecx_t &e, const matx_t &E) {
  // const real_t lambda = 10.0;  // LM damping term
  const real_t lambda = 0.001;  // LM damping term

  // Form weight matrix
  // W = diag(repmat(sigma, data->nb_measurements, 1));

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
    struct timespec t_start = tic();
    const vecx_t e = ba_residuals(data);
    const matx_t E = ba_jacobian(data);
    ba_update(data, e, E);

    const real_t cost = ba_cost(e);
    printf("- iter[%d] cost[%.4e] time: %fs\n", iter, cost, toc(&t_start));

    // Termination criteria
    real_t cost_diff = fabs(cost - cost_prev);
    if (cost_diff < 1.0e-3) {
      printf("Done!\n");
      break;
    }
    cost_prev = cost;
  }
}

int main(int argc, char **argv) {
  if (argc != 2) {
    print_usage();
    return -1;
  }

  struct timespec t_start = tic();
  ba_data_t data{std::string{argv[1]}};
  ba_solve(data);
  printf("time taken: %fs\n", toc(&t_start));
  printf("nb_frames: %d\n", data.nb_frames);
  printf("nb_points: %d\n", data.nb_points);
  printf("\n");

  // Calculate Hessian
  const matx_t E = ba_jacobian(data);
  matx_t H = E.transpose() * E;

  // Find rank(H)
  Eigen::FullPivLU<matx_t> LU(H);
  printf("rank(H): %ld\n", LU.rank());
  printf("rows(H): %ld\n", H.rows());
  printf("det(H): %f\n", LU.determinant());
  printf("Is H invertible?: %d\n", ((LU.isInvertible()) ? 1 : -1));
  printf("\n");

  // Damp Hessian using Levenberg-Marquardt dampening
  // Eq (2.27) in http://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf
  double lambda = 1e-4;
  matx_t H_diag = H.diagonal().asDiagonal();
  matx_t H_damped = H + lambda * H_diag;

  // Check rank(H_damped)
  Eigen::FullPivLU<matx_t> LU_damped(H_damped);
  printf("rank(H_damped): %ld\n", LU_damped.rank());
  printf("rows(H_damped): %ld\n", H_damped.rows());
  printf("det(H_damped): %f\n", LU_damped.determinant());
  printf("Is H_damped invertible?: %d\n", ((LU_damped.isInvertible()) ? 1 : -1));

  return 0;
}
