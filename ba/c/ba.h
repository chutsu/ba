#ifndef BA_H
#define BA_H

#include <stdio.h>
#include <string.h>

#include "util.h"

struct keypoints_t {
  real_t **data;
  int size;
} typedef keypoints_t;

struct ba_data_t {
  /* real_t cam_K[3 * 3]; */
  real_t cam[4];  /* fx, fy, cx, cy */

  int nb_frames;
  real_t **cam_poses;
  real_t **target_pose;

  keypoints_t **keypoints;
  int **point_ids;

  real_t **points;
  int nb_points;
} typedef ba_data_t;

static void load_camera(const char *data_path, real_t cam[4]) {
  /* Setup csv path */
  char cam_csv[1000] = {0};
  strcat(cam_csv, data_path);
  strcat(cam_csv, "/camera.csv");

  /* Parse csv file */
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **cam_K = csv_data(cam_csv, &nb_rows, &nb_cols);
  if (cam_K == NULL) {
    FATAL("Failed to load csv file [%s]!", cam_csv);
  }
  if (nb_rows != 3 || nb_cols != 3) {
    LOG_ERROR("Error while parsing camera file [%s]!", cam_csv);
    LOG_ERROR("-- Expected 3 rows got %d instead!", nb_rows);
    LOG_ERROR("-- Expected 3 cols got %d instead!", nb_cols);
    FATAL("Invalid camera file [%s]!", cam_csv);
  }

  /* Set cam */
  cam[0] = cam_K[0][0];
  cam[1] = cam_K[1][1];
  cam[2] = cam_K[0][2];
  cam[3] = cam_K[1][2];

  /* Free cam_K */
  for (int i = 0; i < nb_rows; i++) {
    free(cam_K[i]);
  }
  free(cam_K);
}

static real_t **load_poses(const char *csv_path, int *nb_poses) {
  assert(csv_path != NULL);
  assert(nb_poses != NULL);

  FILE *csv_file = fopen(csv_path, "r");
  char line[MAX_LINE_LENGTH] = {0};
  *nb_poses = dsv_rows(csv_path);
  real_t **poses = malloc(sizeof(real_t *) * *nb_poses);

  int pose_idx = 0;
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[MAX_LINE_LENGTH] = {0};
    real_t data[7] = {0};
    int index = 0;
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        data[index] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        index++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    poses[pose_idx] = malloc(sizeof(real_t) * 7);
    poses[pose_idx][0] = data[0];
    poses[pose_idx][1] = data[1];
    poses[pose_idx][2] = data[2];
    poses[pose_idx][3] = data[3];
    poses[pose_idx][4] = data[4];
    poses[pose_idx][5] = data[5];
    poses[pose_idx][6] = data[6];

    pose_idx++;
  }
  fclose(csv_file);

  return poses;
}

static real_t **load_camera_poses(const char *data_path, int *nb_cam_poses) {
  char cam_poses_csv[1000] = {0};
  strcat(cam_poses_csv, data_path);
  strcat(cam_poses_csv, "/camera_poses.csv");
  return load_poses(cam_poses_csv, nb_cam_poses);
}

static real_t **load_target_pose(const char *data_path) {
  char target_pose_csv[1000] = {0};
  strcat(target_pose_csv, data_path);
  strcat(target_pose_csv, "/target_pose.csv");

  int nb_poses = 0;
  return load_poses(target_pose_csv, &nb_poses);
}

void keypoints_free(keypoints_t *keypoints) {
  for (int i = 0; i < keypoints->size; i++) {
    free(keypoints->data[i]);
  }
  free(keypoints->data);
  free(keypoints);
}

void keypoints_print(const keypoints_t *keypoints) {
  printf("nb_keypoints: %d\n", keypoints->size);
  printf("keypoints:\n");
  for (int i = 0; i < keypoints->size; i++) {
    printf("-- (%f, %f)\n", keypoints->data[i][0], keypoints->data[i][1]);
  }
}

static keypoints_t *parse_keypoints_line(char *line) {
  keypoints_t *keypoints = calloc(1, sizeof(keypoints_t));
  keypoints->data = NULL;
  keypoints->size = 0;

  char entry[100] = {0};
  int kp_ready = 0;
  real_t kp[2] = {0};
  int kp_index = 0;

  /* Parse line */
  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      /* Initialize keypoints */
      if (keypoints->data == NULL) {
        size_t array_size = strtod(entry, NULL);
        keypoints->data = calloc(array_size, sizeof(real_t *));
        keypoints->size = array_size / 2.0;

      } else { /* Parse keypoint */
        if (kp_ready == 0) {
          kp[0] = strtod(entry, NULL);
          kp_ready = 1;

        } else {
          kp[1] = strtod(entry, NULL);
          keypoints->data[kp_index] = malloc(sizeof(real_t) * 2);
          keypoints->data[kp_index][0] = kp[0];
          keypoints->data[kp_index][1] = kp[1];

          kp_ready = 0;
          kp_index++;
        }
      }

      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return keypoints;
}

static keypoints_t **load_keypoints(const char *data_path, int *nb_frames) {
  char keypoints_csv[1000] = {0};
  strcat(keypoints_csv, data_path);
  strcat(keypoints_csv, "/keypoints.csv");

  FILE *csv_file = fopen(keypoints_csv, "r");
  *nb_frames = dsv_rows(keypoints_csv);
  keypoints_t **keypoints = calloc(*nb_frames, sizeof(keypoints_t *));

  char line[1024] = {0};
  int frame_idx = 0;
  while (fgets(line, 1024, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    keypoints[frame_idx] = parse_keypoints_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return keypoints;
}

static real_t **load_points(const char *data_path, int *nb_points) {
  char points_csv[1000] = {0};
  strcat(points_csv, data_path);
  strcat(points_csv, "/points.csv");

  /* Initialize memory for points */
  *nb_points = dsv_rows(points_csv);
  real_t **points = malloc(sizeof(real_t *) * *nb_points);
  for (int i = 0; i < *nb_points; i++) {
    points[i] = malloc(sizeof(real_t) * 3);
  }

  /* Load file */
  FILE *infile = fopen(points_csv, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Loop through data */
  char line[1024] = {0};
  size_t len_max = 1024;
  int point_idx = 0;
  int col_idx = 0;

  while (fgets(line, len_max, infile) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        points[point_idx][col_idx] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    point_idx++;
  }

  /* Cleanup */
  fclose(infile);

  return points;
}

static int **load_point_ids(const char *data_path, int *nb_points) {
  char csv_path[1000] = {0};
  strcat(csv_path, data_path);
  strcat(csv_path, "/point_ids.csv");
  return load_iarrays(csv_path, nb_points);
}

ba_data_t *ba_load_data(const char *data_path) {
  ba_data_t *data = malloc(sizeof(ba_data_t));

  int nb_ids = 0;
  load_camera(data_path, data->cam);
  data->cam_poses = load_camera_poses(data_path, &data->nb_frames);
  data->target_pose = load_target_pose(data_path);
  data->keypoints = load_keypoints(data_path, &data->nb_frames);
  data->point_ids = load_point_ids(data_path, &nb_ids);
  data->points = load_points(data_path, &data->nb_points);

  return data;
}

void ba_data_free(ba_data_t *data) {
  /* Camera poses */
  for (int k = 0; k < data->nb_frames; k++) {
    free(data->cam_poses[k]);
  }
  free(data->cam_poses);

  /* Target pose */
  free(data->target_pose[0]);
  free(data->target_pose);

  /* Keypoints */
  for (int i = 0; i < data->nb_frames; i++) {
    keypoints_free(data->keypoints[i]);
  }
  free(data->keypoints);

  /* Point IDs */
  for (int i = 0; i < data->nb_frames; i++) {
    free(data->point_ids[i]);
  }
  free(data->point_ids);

  /* Points */
  for (int i = 0; i < data->nb_points; i++) {
    free(data->points[i]);
  }
  free(data->points);

  free(data);
}

int ba_residual_size(ba_data_t *data) {
  /* Calculate residual size */
  int r_size = 0;
  for (int k = 0; k < data->nb_frames; k++) {
    r_size += data->point_ids[k][0];
  }
  r_size = r_size * 2;
  /* ^ Scale 2 because each pixel error are size 2 */

  return r_size;
}

real_t *ba_residuals(ba_data_t *data, int *r_size) {
  /* Initialize memory for residuals */
  *r_size = ba_residual_size(data);
  real_t *r = calloc(*r_size, sizeof(real_t));

  /* Target pose */
  real_t T_WT[4 * 4] = {0};
  tf(data->target_pose[0], T_WT);

  /* Loop over time */
  int res_idx = 0; /* Residual index */
  for (int k = 0; k < data->nb_frames; k++) {
    /* Form camera pose */
    real_t T_WC[4 * 4] = {0};
    tf(data->cam_poses[k], T_WC);

    /* Invert camera pose T_WC to T_CW */
    real_t T_CW[4 * 4] = {0};
    tf_inv(T_WC, T_CW);

    /* Get point ids and measurements at time step k */
    const int nb_ids = data->point_ids[k][0];
    const int *point_ids = &data->point_ids[k][1];

    for (int i = 0; i < nb_ids; i++) {
      /* Get point in world frame */
      const int id = point_ids[i];
      const real_t *p_W = data->points[id];

      /* Transform point in world frame to camera frame */
      real_t p_C[3] = {0};
      tf_point(T_CW, p_W, p_C);

      /* Project point in camera frame down to image plane */
      real_t z_hat[2] = {0};
      pinhole_project(data->cam, p_C, z_hat);

      /* Calculate reprojection error */
      const real_t *z = data->keypoints[k]->data[i];
      r[res_idx] = z[0] - z_hat[0];
      r[res_idx + 1] = z[1] - z_hat[1];
      res_idx += 2;
    }
  }

  return r;
}

static void J_intrinsics_point(const real_t cam[4], real_t J[2 * 2]) {
  /* J = [fx, 0,  */
  /* 		  0, fy]; */
  const real_t fx = cam[0];
  const real_t fy = cam[1];
  zeros(J, 2, 2);
  J[0] = fx;
  J[3] = fy;
}

static void J_project(const real_t p_C[3], real_t J[2 * 3]) {
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];

  /* J = [1 / z, 0, -x / z^2, */
  /* 		  0, 1 / z, -y / z^2]; */
  zeros(J, 2, 3);
  J[0] = 1.0 / z;
  J[2] = -x / (z * z);
  J[4] = 1.0 / z;
  J[5] = -y / (z * z);
}

static void J_camera_rotation(const real_t q_WC[4],
                              const real_t r_WC[3],
                              const real_t p_W[3],
                              real_t J[3 * 3]) {
  /* Convert quaternion to rotatoin matrix */
  real_t C_WC[3 * 3] = {0};
  quat2rot(q_WC, C_WC);

  /* J = C_WC * skew(p_W - r_WC); */
  real_t C_CW[3 * 3] = {0};
  mat_transpose(C_WC, 3, 3, C_CW);

  real_t x[3] = {0};
  vec_sub(p_W, r_WC, x, 3);

  real_t S[3 * 3] = {0};
  skew(x, S);

  cblas_dot(C_CW, 3, 3, S, 3, 3, J);
}

static void J_camera_translation(const real_t q_WC[4], real_t J[3 * 3]) {
  /* Convert quaternion to rotatoin matrix */
  real_t C_WC[3 * 3] = {0};
  quat2rot(q_WC, C_WC);

  /* J = -C_CW */
  mat_transpose(C_WC, 3, 3, J);
  mat_scale(J, 3, 3, -1.0);
}

static void J_target_point(const real_t q_WC[4], real_t J[3 * 3]) {
  /* Convert quaternion to rotatoin matrix */
  real_t C_WC[3 * 3] = {0};
  quat2rot(q_WC, C_WC);

  /* J = C_CW */
  mat_transpose(C_WC, 3, 3, J);
}

real_t *ba_jacobian(ba_data_t *data, int *J_rows, int *J_cols) {
  /* Initialize memory for jacobian */
  *J_rows = ba_residual_size(data);
  *J_cols = (data->nb_frames * 6) + (data->nb_points * 3);
  real_t *J = calloc(*J_rows * *J_cols, sizeof(real_t));
  zeros(J, *J_rows, *J_cols);

  /* Loop over camera poses */
  int pose_idx = 0;
  int meas_idx = 0;

  for (int k = 0; k < data->nb_frames; k++) {
    /* Form camera pose */
    real_t T_WC[4 * 4] = {0};
    real_t q_WC[4] = {0};
    real_t r_WC[3] = {0};
    tf(data->cam_poses[k], T_WC);
    tf_quat_get(T_WC, q_WC);
    tf_trans_get(T_WC, r_WC);

    /* Invert T_WC to T_CW */
    real_t T_CW[4 * 4] = {0};
    tf_inv(T_WC, T_CW);

    /* Get point ids and measurements at time step k */
    const int nb_ids = data->point_ids[k][0];
    const int *point_ids = &data->point_ids[k][1];

    /* Loop over observations at time k */
    for (int i = 0; i < nb_ids; i++) {
      /* Get point in world frame */
      const int id = point_ids[i];
      const real_t *p_W = data->points[id];

      /* Transform point in world frame to camera frame */
      real_t p_C[3] = {0};
      tf_point(T_CW, p_W, p_C);

      /* Camera pose jacobian */
      /* -- Setup row start, row end, column start and column end */
      const int rs = meas_idx * 2;
      const int re = rs + 1;
      int cs = pose_idx * 6;
      int ce = cs + 5;

      /* -- Form jacobians */
      real_t J_K[2 * 2] = {0};
      real_t J_P[2 * 3] = {0};
      real_t J_h[2 * 3] = {0};
      J_intrinsics_point(data->cam, J_K);
      J_project(p_C, J_P);
      cblas_dot(J_K, 2, 2, J_P, 2, 3, J_h);

      /* -- J_cam_rot = -1 * J_h * J_C; */
      real_t J_C[3 * 3] = {0};
      real_t J_cam_rot[2 * 3] = {0};
      J_camera_rotation(q_WC, r_WC, p_W, J_C);
      cblas_dot(J_h, 2, 3, J_C, 3, 3, J_cam_rot);
      mat_scale(J_cam_rot, 2, 3, -1);

      /* -- J_cam_pos = -1 * J_h * J_r; */
      real_t J_r[3 * 3] = {0};
      real_t J_cam_pos[2 * 3] = {0};
      J_camera_translation(q_WC, J_r);
      cblas_dot(J_h, 2, 3, J_r, 3, 3, J_cam_pos);
      mat_scale(J_cam_pos, 2, 3, -1);

      /* -- Fill in the big jacobian */
      mat_block_set(J, *J_cols, rs, cs, re, cs + 2, J_cam_rot);
      mat_block_set(J, *J_cols, rs, cs + 3, re, ce, J_cam_pos);

      /* Point jacobian */
      /* -- Setup row start, row end, column start and column end */
      cs = (data->nb_frames * 6) + point_ids[i] * 3;
      ce = cs + 2;
      /* -- Form jacobians */
      real_t J_p[3 * 3] = {0};
      J_target_point(q_WC, J_p);
      /* -- J_point = -1 * J_h * J_target_point(q_WC); */
      real_t J_point[2 * 3] = {0};
      cblas_dot(J_h, 2, 3, J_p, 3, 3, J_point);
      mat_scale(J_point, 2, 3, -1);
      /* -- Fill in the big jacobian */
      mat_block_set(J, *J_cols, rs, cs, re, ce, J_point);

      meas_idx++;
    }
    pose_idx++;
  }

  return J;
}

void ba_update(ba_data_t *data, real_t *dx) {
  /* Update camera poses */
  for (int k = 0; k < data->nb_frames; k++) {
    const int s = k * 6;

    /* Update camera rotation */
    /* dq = quat_delta(dalpha) */
    /* q_WC_k = quat_mul(dq, q_WC_k) */
    real_t *cam_pose = data->cam_poses[k];
    const real_t dalpha[3] = {dx[s], dx[s + 1], dx[s + 2]};
    real_t dq[4] = {0};
    real_t q_WC[4] = {cam_pose[0], cam_pose[1], cam_pose[2], cam_pose[3]};
    real_t q_new[4] = {0};
    quat_delta(dalpha, dq);
    quat_mul(dq, q_WC, q_new);
    cam_pose[0] = q_new[0];
    cam_pose[1] = q_new[1];
    cam_pose[2] = q_new[2];
    cam_pose[3] = q_new[3];

    /* Update camera position */
    /* r_WC_k += dr_WC */
    const real_t dr_WC[3] = {dx[s + 3], dx[s + 4], dx[s + 5]};
    cam_pose[4] += dr_WC[0];
    cam_pose[5] += dr_WC[1];
    cam_pose[6] += dr_WC[2];
  }

  /* Update points */
  for (int i = 0; i < data->nb_points; i++) {
    const int s = (data->nb_frames * 6) + (i * 3);
    const real_t dp_W[3] = {dx[s], dx[s + 1], dx[s + 2]};
    data->points[i][0] += dp_W[0];
    data->points[i][1] += dp_W[1];
    data->points[i][2] += dp_W[2];
  }
}

real_t ba_cost(const real_t *e, const int length) {
  /* cost = 0.5 * e' * e */
  real_t cost = 0.0;
  cblas_dot(e, 1, length, e, length, 1, &cost);
  return cost * 0.5;
}

static real_t calc_reproj_error(const real_t *e, const int length) {
  real_t sse = 0;
  for (int i = 0; i < length; i += 2) {
    real_t err_x = e[i];
    real_t err_y = e[i + 1];
    real_t err_norm = sqrt(err_x * err_x + err_y * err_y);
    sse += err_norm * err_norm;
  }

  real_t nb_reproj_errors = length / 2.0;
  real_t mse = sse / nb_reproj_errors;
  real_t rmse = sqrt(mse);

  return rmse;
}

void ba_solve(ba_data_t *data) {
  int max_iter = 10;
  real_t lambda = 1e-4;

  int e_size = 0;
  real_t *e = ba_residuals(data, &e_size);
  real_t cost_prev = ba_cost(e, e_size);
  free(e);

  for (int iter = 0; iter < max_iter; iter++) {
    /* Jacobians */
    int E_rows = 0;
    int E_cols = 0;
    real_t *E = ba_jacobian(data, &E_rows, &E_cols);

    /* Solve Gauss-Newton system [H dx = g]: Solve for dx */
    /* -- Calculate L.H.S of Gauss-Newton: H = (E' * W * E); */
    real_t *E_t = mat_new(E_cols, E_rows);
    real_t *H = mat_new(E_cols, E_cols);
    mat_transpose(E, E_rows, E_cols, E_t);
    cblas_dot(E_t, E_cols, E_rows, E, E_rows, E_cols, H);
    /* -- Apply Levenberg-Marquardt damping: H = H + lambda * H_diag */
    for (int i = 0; i < E_cols; i++) {
      H[(i * E_cols) + i] += lambda * H[(i * E_cols) + i];
    }
    /* -- Calculate R.H.S of Gauss-Newton: g = -E' * W * e; */
    real_t *g = vec_new(E_cols);
    mat_scale(E_t, E_cols, E_rows, -1.0);
    e = ba_residuals(data, &e_size);
    cblas_dot(E_t, E_cols, E_rows, e, e_size, 1, g);
    free(e);
    free(E);
    free(E_t);
    /* -- Solve linear system: H * dx = g */
    real_t *dx = vec_new(E_cols);
    /* chol_solve(H, g, dx, E_cols); */
    lapack_chol_solve(H, g, dx, E_cols);
    free(H);
    free(g);

    /* Update */
    ba_update(data, dx);
    free(dx);

    /* Calculate cost */
    e = ba_residuals(data, &e_size);
    real_t cost = ba_cost(e, e_size);
    const real_t dcost = cost - cost_prev;
    const real_t rms_reproj_error = calc_reproj_error(e, e_size);
    free(e);

    printf("iter: [%d]  ", iter);
    printf("lambda: %.2e  ", lambda);
    printf("cost: %.4e  ", cost);
    printf("dcost: %.2e  ", dcost);
    printf("rms reproj error: %.2f\n", rms_reproj_error);

    /* Termination criteria */
    if (fabs(dcost) < 1.0e-1) {
      printf("Done!\n");
      break;
    }

    /* Update lambda */
    if (dcost < 0) {
      lambda /= 10.0;
      cost_prev = cost;

    } else {
      lambda *= 10.0;

      /* Restore previous state because update failed */
      for (int i = 0; i < E_cols; i++)
        dx[i] *= -1;
      ba_update(data, dx);
    }
  }
}

#endif // BA_H
