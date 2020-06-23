#include "munit.hpp"
#include "ba.hpp"

#define TEST_DATA "./data"

int check_J_cam_pose(const mat3_t &cam_K,
                     const mat4_t &T_WC,
                     const vec3_t &p_W,
                     const mat_t<2, 6> &J_cam_pose,
                     const double step_size = 1e-3,
                     const double threshold = 1e-2) {
  const vec2_t z{0.0, 0.0};
  const vec4_t hp_W = p_W.homogeneous();

  // Perturb rotation
  matx_t fdiff = zeros(2, 6);
  for (int i = 0; i < 3; i++) {
    // Forward difference
    const mat4_t T_WC_fd = tf_perturb_rot(T_WC, step_size, i);
    const mat4_t T_CW_fd = T_WC_fd.inverse();
    const vec3_t p_C_fd = (T_CW_fd * hp_W).head(3);
    const vec3_t x_fd{p_C_fd(0) / p_C_fd(2), p_C_fd(1) / p_C_fd(2), 1.0};
    const vec2_t z_fd = (cam_K * x_fd).head(2);
    const vec2_t e_fd = z - z_fd;

    // Backward difference
    const mat4_t T_WC_bd = tf_perturb_rot(T_WC, -step_size, i);
    const mat4_t T_CW_bd = T_WC_bd.inverse();
    const vec3_t p_C_bd = (T_CW_bd * hp_W).head(3);
    const vec3_t x_bd{p_C_bd(0) / p_C_bd(2), p_C_bd(1) / p_C_bd(2), 1.0};
    const vec2_t z_bd = (cam_K * x_bd).head(2);
    const vec2_t e_bd = z - z_bd;

    fdiff.block(0, i, 2, 1) = (e_fd - e_bd) / (2 * step_size);
  }

  // Perturb translation
  for (int i = 0; i < 3; i++) {
    // Forward difference
    const mat4_t T_WC_fd = tf_perturb_trans(T_WC, step_size, i);
    const mat4_t T_CW_fd = T_WC_fd.inverse();
    const vec3_t p_C_fd = (T_CW_fd * hp_W).head(3);
    const vec3_t x_fd{p_C_fd(0) / p_C_fd(2), p_C_fd(1) / p_C_fd(2), 1.0};
    const vec2_t z_fd = (cam_K * x_fd).head(2);
    const vec2_t e_fd = z - z_fd;

    // Backward difference
    const mat4_t T_WC_bd = tf_perturb_trans(T_WC, -step_size, i);
    const mat4_t T_CW_bd = T_WC_bd.inverse();
    const vec3_t p_C_bd = (T_CW_bd * hp_W).head(3);
    const vec3_t x_bd{p_C_bd(0) / p_C_bd(2), p_C_bd(1) / p_C_bd(2), 1.0};
    const vec2_t z_bd = (cam_K * x_bd).head(2);
    const vec2_t e_bd = z - z_bd;

    fdiff.block(0, i + 3, 2, 1) = (e_fd - e_bd) / (2 * step_size);
  }

  return check_jacobian("J_cam_pose", fdiff, J_cam_pose, threshold, true);
}

int check_J_point(const mat3_t &cam_K,
                  const mat4_t &T_WC,
                  const vec3_t &p_W,
                  const mat_t<2, 3> &J_point,
                  const double step_size = 1e-10,
                  const double threshold = 1e-2) {
  const vec2_t z{0.0, 0.0};
  const mat4_t T_CW = T_WC.inverse();
  matx_t fdiff = zeros(2, 3);
  mat3_t dr = I(3) * step_size;

  // Perturb landmark
  for (int i = 0; i < 3; i++) {
    // Forward difference
    const vec3_t p_W_fd = p_W + dr.col(i);
    const vec4_t hp_W_fd = p_W_fd.homogeneous();
    const vec3_t p_C_fd = (T_CW * hp_W_fd).head(3);
    const vec3_t x_fd{p_C_fd(0) / p_C_fd(2), p_C_fd(1) / p_C_fd(2), 1.0};
    const vec2_t z_fd = (cam_K * x_fd).head(2);
    const vec2_t e_fd = z - z_fd;

    // Backward difference
    const vec3_t p_W_bd = p_W - dr.col(i);
    const vec4_t hp_W_bd = p_W_bd.homogeneous();
    const vec3_t p_C_bd = (T_CW * hp_W_bd).head(3);
    const vec3_t x_bd{p_C_bd(0) / p_C_bd(2), p_C_bd(1) / p_C_bd(2), 1.0};
    const vec2_t z_bd = (cam_K * x_bd).head(2);
    const vec2_t e_bd = z - z_bd;

    fdiff.block(0, i, 2, 1) = (e_fd - e_bd) / (2 * step_size);
  }

  return check_jacobian("J_point", fdiff, J_point, threshold, true);
}

// int test_parse_keypoints_line() {
//   std::string line = "4,1,2,3,4\n";
//   keypoints_t keypoints = parse_keypoints_line(line.c_str());
//
//   keypoints_print(keypoints);
//   MU_CHECK(keypoints.size() == 2);
//   MU_CHECK(fltcmp(keypoints[0](0), 1.0) == 0);
//   MU_CHECK(fltcmp(keypoints[0](1), 2.0) == 0);
//   MU_CHECK(fltcmp(keypoints[1](0), 3.0) == 0);
//   MU_CHECK(fltcmp(keypoints[1](1), 4.0) == 0);
//
//   return 0;
// }

int test_load_keypoints() {
  std::vector<keypoints_t> keypoints = load_keypoints(TEST_DATA);

  for (const auto &kp : keypoints) {
    MU_CHECK(kp.size() > 0);
    /* printf("frame[%d]\n", i); */
    /* keypoints_print(keypoints[i]); */
  }

  return 0;
}

int test_ba_residuals() {
  ba_data_t data{TEST_DATA};

  vecx_t r = ba_residuals(data);
  // for (int i = 0; i < r.rows(); i++) {
  //   MU_CHECK(r[i] < 0.01);
  // }

  const double cost = ba_cost(r);
  printf("Cost: %f\n", cost);

  return 0;
}

int test_ba_jacobian() {
  ba_data_t data{TEST_DATA};
  // for (const auto pose : data.cam_poses) {
  //   std::cout << pose.q.w() << ", ";
  //   std::cout << pose.q.x() << ", ";
  //   std::cout << pose.q.y() << ", ";
  //   std::cout << pose.q.z() << std::endl;
  // }

  matx_t J = ba_jacobian(data);
  mat2csv("/tmp/J.csv", J);

  return 0;
}

int test_ba_update() {
  ba_data_t data{TEST_DATA};
  const vecx_t e = ba_residuals(data);
  const matx_t E = ba_jacobian(data);
  ba_update(data, e, E);

  return 0;
}

int test_ba_cost() {
  vecx_t e{5};
  e << 1.0, 2.0, 3.0, 4.0, 5.0;
  const double cost = ba_cost(e);

  printf("Cost: %f\n", cost);
  MU_CHECK(fltcmp(cost, 27.50) == 0);

  return 0;
}

int test_ba_solve() {
  struct timespec t_start = tic();
  ba_data_t data{TEST_DATA};
  ba_solve(data);
  printf("time taken: %fs\n", toc(&t_start));
  printf("nb_frames: %d\n", data.nb_frames);
  printf("nb_points: %d\n", data.nb_points);

  return 0;
}

void test_suite() {
  // MU_ADD_TEST(test_parse_keypoints_line);
  MU_ADD_TEST(test_load_keypoints);
  MU_ADD_TEST(test_ba_residuals);
  MU_ADD_TEST(test_ba_jacobian);
  MU_ADD_TEST(test_ba_update);
  MU_ADD_TEST(test_ba_cost);
  MU_ADD_TEST(test_ba_solve);
}

MU_RUN_TESTS(test_suite);
