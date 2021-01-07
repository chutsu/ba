#include "munit.h"
#include "ba.h"

#define TEST_BA_DATA_GT "test_data/data_gnd"
#define TEST_BA_DATA "test_data/data_noisy"
#define TEST_BA_JAC "test_data/ba_jacobian.csv"
#define TEST_BA_UPDATE_DX "test_data/ba_update_dx.csv"

#define STEP_SIZE 1e-8
#define THRESHOLD 1e-3

static void
project(const real_t *K, const real_t *T_WC, const real_t *p_W, real_t *z) {
  /* Invert camera pose to T_CW */
  real_t T_CW[4 * 4] = {0};
  tf_inv(T_WC, T_CW);

  /* Transform point from world frame to camera frame */
  real_t p_C[3] = {0};
  tf_point(T_CW, p_W, p_C);

  /* Project */
  const real_t x = p_C[0] / p_C[2];
  const real_t y = p_C[1] / p_C[2];

  /* Scale and center */
  const real_t fx = K[0];
  const real_t fy = K[4];
  const real_t cx = K[2];
  const real_t cy = K[5];
  z[0] = fx * x + cx;
  z[1] = fy * y + cy;
}

static int check_J_cam_pose(const real_t *cam_K,
                            const real_t *T_WC,
                            const real_t *p_W,
                            const real_t *J_cam_pose) {
  const real_t step_size = 1e-8;
  const real_t threshold = 1e-2;
  const real_t z[2] = {0.0, 0.0};
  real_t fdiff[2 * 6] = {0.0};

  // Perturb rotation
  for (int i = 0; i < 3; i++) {
    // Forward difference
    // -- Perturb rotation
    real_t T_WC_fd[4 * 4] = {0};
    mat_copy(T_WC, 4, 4, T_WC_fd);
    tf_perturb_rot(T_WC_fd, step_size, i);
    // -- Project landmark to image plane
    real_t z_fd[2] = {0};
    project(cam_K, T_WC_fd, p_W, z_fd);
    // -- Calculate reprojection error
    real_t e_fd[2] = {0};
    e_fd[0] = z[0] - z_fd[0];
    e_fd[1] = z[1] - z_fd[1];

    // Backward difference
    // -- Perturb rotation
    real_t T_WC_bd[4 * 4] = {0};
    mat_copy(T_WC, 4, 4, T_WC_bd);
    tf_perturb_rot(T_WC_bd, -step_size, i);
    // -- Project landmark to image plane
    real_t z_bd[2] = {0};
    project(cam_K, T_WC_bd, p_W, z_bd);
    // -- Calculate reprojection error
    real_t e_bd[2] = {0};
    e_bd[0] = z[0] - z_bd[0];
    e_bd[1] = z[1] - z_bd[1];

    // Set finite difference
    fdiff[i] = (e_fd[0] - e_bd[0]) / (2 * step_size);
    fdiff[i + 6] = (e_fd[1] - e_bd[1]) / (2 * step_size);
  }

  // Perturb translation
  for (int i = 0; i < 3; i++) {
    // Forward difference
    // -- Perturb translation
    real_t T_WC_fd[4 * 4] = {0};
    mat_copy(T_WC, 4, 4, T_WC_fd);
    tf_perturb_trans(T_WC_fd, step_size, i);
    // -- Project landmark to image plane
    real_t z_fd[2] = {0};
    project(cam_K, T_WC_fd, p_W, z_fd);
    // -- Calculate reprojection error
    real_t e_fd[2] = {z[0] - z_fd[0], z[1] - z_fd[1]};

    // Backward difference
    // -- Perturb translation
    real_t T_WC_bd[4 * 4] = {0};
    mat_copy(T_WC, 4, 4, T_WC_bd);
    tf_perturb_trans(T_WC_bd, -step_size, i);
    // -- Project landmark to image plane
    real_t z_bd[2] = {0};
    project(cam_K, T_WC_bd, p_W, z_bd);
    // -- Calculate reprojection error
    real_t e_bd[2] = {z[0] - z_bd[0], z[1] - z_bd[1]};

    // Set finite difference
    fdiff[i + 3] = (e_fd[0] - e_bd[0]) / (2 * step_size);
    fdiff[i + 9] = (e_fd[1] - e_bd[1]) / (2 * step_size);
  }

  return check_jacobian("J_cam_pose", fdiff, J_cam_pose, 2, 6, threshold, 1);
}

static int check_J_landmark(const real_t *cam_K,
                            const real_t *T_WC,
                            const real_t *p_W,
                            const real_t *J_landmark) {
  const real_t step_size = 1e-8;
  const real_t threshold = 1e-2;
  const real_t z[2] = {0.0, 0.0};
  real_t fdiff[2 * 6] = {0.0};

  // Perturb landmark
  for (int i = 0; i < 3; i++) {
    // Forward difference
    // -- Perturb landmark
    real_t p_W_fd[3] = {p_W[0], p_W[1], p_W[2]};
    p_W_fd[i] += step_size;
    // -- Project landmark to image plane
    real_t z_fd[2] = {0};
    project(cam_K, T_WC, p_W_fd, z_fd);
    // -- Calculate reprojection error
    real_t e_fd[2] = {z[0] - z_fd[0], z[1] - z_fd[1]};

    // Backward difference
    // -- Perturb landmark
    real_t p_W_bd[3] = {p_W[0], p_W[1], p_W[2]};
    p_W_bd[i] -= step_size;
    // -- Project landmark to image plane
    real_t z_bd[2] = {0};
    project(cam_K, T_WC, p_W_bd, z_bd);
    // -- Calculate reprojection error
    real_t e_bd[2] = {z[0] - z_bd[0], z[1] - z_bd[1]};

    // Set finite difference
    fdiff[i] = (e_fd[0] - e_bd[0]) / (2 * step_size);
    fdiff[i + 3] = (e_fd[1] - e_bd[1]) / (2 * step_size);
  }

  return check_jacobian("J_landmark", fdiff, J_landmark, 2, 3, threshold, 1);
}

int test_load_camera() {
  real_t K[3 * 3] = {0};
  load_camera(TEST_BA_DATA, K);
  print_matrix("K", K, 3, 3);

  return 0;
}

int test_parse_keypoints_line() {
  keypoints_t *keypoints = parse_keypoints_line("4,1,2,3,4\n");

  /* keypoints_print(keypoints); */
  MU_CHECK(keypoints->size == 2);
  MU_CHECK(fltcmp(keypoints->data[0][0], 1.0) == 0);
  MU_CHECK(fltcmp(keypoints->data[0][1], 2.0) == 0);
  MU_CHECK(fltcmp(keypoints->data[1][0], 3.0) == 0);
  MU_CHECK(fltcmp(keypoints->data[1][1], 4.0) == 0);

  keypoints_free(keypoints);

  return 0;
}

int test_load_keypoints() {
  int nb_frames = 0;
  keypoints_t **keypoints = load_keypoints(TEST_BA_DATA, &nb_frames);

  for (int i = 0; i < nb_frames; i++) {
    printf("frame: %d ", i);
    printf("nb_keypoints: %d\n", keypoints[i]->size);
    MU_CHECK(keypoints[i] != NULL);
    MU_CHECK(keypoints[i]->size > 0);
    /* printf("frame[%d]\n", i); */
    /* keypoints_print(keypoints[i]); */
    keypoints_free(keypoints[i]);
  }
  free(keypoints);

  return 0;
}

int test_ba_load_data() {
  ba_data_t *data = ba_load_data(TEST_BA_DATA);
  ba_data_free(data);
  return 0;
}

int test_ba_residuals() {
  ba_data_t *data = ba_load_data(TEST_BA_DATA);

  int r_size = 0;
  real_t *r = ba_residuals(data, &r_size);
  /* for (int i = 0; i < r_size; i++) { */
  /*   MU_CHECK(r[i] < 0.01); */
  /* } */

  const real_t cost = ba_cost(r, r_size);
  printf("Cost: %e\n", cost);

  ba_data_free(data);
  free(r);
  return 0;
}

int test_J_cam_pose() {
  /* Setup camera intrinsics */
  real_t cam_K[3 * 3] = {640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0};

  /* Setup camera pose */
  /* -- Rotation -- */
  const real_t roll = deg2rad(-90.0);
  const real_t pitch = deg2rad(0.0);
  const real_t yaw = deg2rad(-90.0);
  const real_t rpy[3] = {roll, pitch, yaw};
  real_t C_WC[3 * 3] = {0};
  real_t q_WC[4] = {0};
  euler321(rpy, C_WC);
  rot2quat(C_WC, q_WC);
  /* -- Translation -- */
  real_t r_WC[3] = {0.1, 0.2, 0.3};
  /* -- Transform -- */
  real_t T_WC[4 * 4] = {0};
  tf_rot_set(T_WC, C_WC);
  tf_trans_set(T_WC, r_WC);
  mat_set(T_WC, 4, 3, 3, 1.0);

  /* Landmark in world frame */
  real_t p_W[3] = {10.0, 0.0, 0.0};

  /* Transform point in world frame to camera frame */
  real_t T_CW[4 * 4] = {0};
  real_t p_C[3] = {0};
  tf_inv(T_WC, T_CW);
  tf_point(T_CW, p_W, p_C);

  /* -- Form jacobians */
  real_t J_K[2 * 2] = {0};
  real_t J_P[2 * 3] = {0};
  real_t J_h[2 * 3] = {0};
  J_intrinsics_point(cam_K, J_K);
  J_project(p_C, J_P);
  dot(J_K, 2, 2, J_P, 2, 3, J_h);

  /* J_cam_rot = -1 * J_K * J_P * J_C; */
  real_t J_C[3 * 3] = {0};
  real_t J_cam_rot[2 * 3] = {0};
  J_camera_rotation(q_WC, r_WC, p_W, J_C);
  dot(J_h, 2, 3, J_C, 3, 3, J_cam_rot);
  mat_scale(J_cam_rot, 2, 3, -1);

  /* J_cam_pos = -1 * J_K * J_P * J_r; */
  real_t J_r[3 * 3] = {0};
  real_t J_cam_pos[2 * 3] = {0};
  J_camera_translation(q_WC, J_r);
  dot(J_h, 2, 3, J_r, 3, 3, J_cam_pos);
  mat_scale(J_cam_pos, 2, 3, -1);

  /* Form J_cam_pose */
  real_t J_cam_pose[2 * 6] = {0};
  mat_block_set(J_cam_pose, 6, 0, 0, 1, 2, J_cam_rot);
  mat_block_set(J_cam_pose, 6, 0, 3, 1, 5, J_cam_pos);

  /* Check jacobians */
  int retval = check_J_cam_pose(cam_K, T_WC, p_W, J_cam_pose);
  MU_CHECK(retval == 0);

  return 0;
}

int test_J_landmark() {
  /* Setup camera intrinsics */
  real_t cam_K[3 * 3] = {640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0};

  /* Setup camera pose */
  /* -- Rotation -- */
  const real_t roll = deg2rad(-90.0);
  const real_t pitch = deg2rad(0.0);
  const real_t yaw = deg2rad(-90.0);
  const real_t rpy[3] = {roll, pitch, yaw};
  real_t C_WC[3 * 3] = {0};
  real_t q_WC[4] = {0};
  euler321(rpy, C_WC);
  rot2quat(C_WC, q_WC);
  /* -- Translation -- */
  real_t r_WC[3] = {0.1, 0.2, 0.3};
  /* -- Transform -- */
  real_t T_WC[4 * 4] = {0};
  tf_rot_set(T_WC, C_WC);
  tf_trans_set(T_WC, r_WC);
  mat_set(T_WC, 4, 3, 3, 1.0);

  /* Landmark in world frame */
  real_t p_W[3] = {10.0, 0.0, 0.0};

  /* Transform point in world frame to camera frame */
  real_t T_CW[4 * 4] = {0};
  real_t p_C[3] = {0};
  tf_inv(T_WC, T_CW);
  tf_point(T_CW, p_W, p_C);

  /* -- Form jacobians */
  real_t J_K[2 * 2] = {0};
  real_t J_P[2 * 3] = {0};
  real_t J_KP[2 * 3] = {0};
  real_t J_L[3 * 3] = {0};
  real_t J_landmark[2 * 3] = {0};
  J_intrinsics_point(cam_K, J_K);
  J_project(p_C, J_P);
  J_target_point(q_WC, J_L);
  dot(J_K, 2, 2, J_P, 2, 3, J_KP);
  dot(J_KP, 2, 3, J_L, 3, 3, J_landmark);
  mat_scale(J_landmark, 2, 3, -1);

  /* Check jacobians */
  int retval = check_J_landmark(cam_K, T_WC, p_W, J_landmark);
  MU_CHECK(retval == 0);

  return 0;
}

int test_ba_jacobian() {
  /* Test function */
  int J_rows = 0;
  int J_cols = 0;
  ba_data_t *data = ba_load_data(TEST_BA_DATA);
  real_t *J = ba_jacobian(data, &J_rows, &J_cols);
  mat_save("/tmp/J0.csv", J, J_rows, J_cols);

  /* #<{(| Load ground truth jacobian |)}># */
  /* int nb_rows = 0.0; */
  /* int nb_cols = 0.0; */
  /* real_t **J_data = csv_data(TEST_BA_JAC, &nb_rows, &nb_cols); */
  /*  */
  /* #<{(| Compare calculated jacobian against ground truth jacobian |)}># */
  /* MU_CHECK(J_rows == nb_rows); */
  /* MU_CHECK(J_cols == nb_cols); */
  /* int index = 0; */
  /* int jac_ok = 1; */

  /*   for (int i = 0; i < nb_rows; i++) { */
  /*     for (int j = 0; j < nb_cols; j++) { */
  /*       if (fabs(J_data[i][j] - J[index]) > 1e-5) { */
  /*         printf("row: [%d] col: [%d] index: [%d] ", i, j, index); */
  /*         printf("expected: [%f] ", J_data[i][j]); */
  /*         printf("got: [%f]\n", J[index]); */
  /*         jac_ok = 0; */
  /*         goto end; */
  /*       } */
  /*       index++; */
  /*     } */
  /*   } */
  /* end: */
  /*   MU_CHECK(jac_ok == 1); */

  /* Clean up */
  free(J);
  ba_data_free(data);
  /* OCTAVE_SCRIPT("scripts/plot_matrix.m /tmp/J.csv"); */

  return 0;
}

int test_ba_update() {
  ba_data_t *data = ba_load_data(TEST_BA_DATA);

  int e_size = 0;
  real_t *e_before = ba_residuals(data, &e_size);
  printf("before:  %f\n", ba_cost(e_before, e_size));

  real_t *dx = load_vector(TEST_BA_UPDATE_DX);
  ba_update(data, dx);

  real_t *e_after = ba_residuals(data, &e_size);
  printf("after:  %f\n", ba_cost(e_after, e_size));

  free(dx);
  free(e_before);
  free(e_after);
  ba_data_free(data);

  return 0;
}

int test_ba_cost() {
  const real_t e[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
  const real_t cost = ba_cost(e, 5);

  printf("Cost: %f\n", cost);
  MU_CHECK(fltcmp(cost, 27.50) == 0);

  return 0;
}

int test_ba_solve() {
  ba_data_t *data = ba_load_data(TEST_BA_DATA);
  struct timespec t_start = tic();
  ba_solve(data);
  printf("time taken: %f\n", toc(&t_start));
  ba_data_free(data);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_load_camera);
  MU_ADD_TEST(test_parse_keypoints_line);
  MU_ADD_TEST(test_load_keypoints);
  MU_ADD_TEST(test_ba_load_data);
  MU_ADD_TEST(test_ba_residuals);
  MU_ADD_TEST(test_J_cam_pose);
  MU_ADD_TEST(test_J_landmark);
  MU_ADD_TEST(test_ba_jacobian);
  MU_ADD_TEST(test_ba_update);
  MU_ADD_TEST(test_ba_cost);
  MU_ADD_TEST(test_ba_solve);
}

MU_RUN_TESTS(test_suite)
