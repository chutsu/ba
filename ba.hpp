#include "util.hpp"

static int check_J_cam_pose(const mat3_t &cam_K,
                            const mat4_t &T_WC,
                            const vec3_t &p_W,
                            const mat_t<2, 6> &J_cam_pose,
                            const real_t step_size = 1e-3,
                            const real_t threshold = 1e-2) {
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

    // Calculate central finite difference
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

    // Calculate central finite difference
    fdiff.block(0, i + 3, 2, 1) = (e_fd - e_bd) / (2 * step_size);
  }

  return check_jacobian("J_cam_pose", fdiff, J_cam_pose, threshold, true);
}

static int check_J_point(const mat3_t &cam_K,
                         const mat4_t &T_WC,
                         const vec3_t &p_W,
                         const mat_t<2, 3> &J_point,
                         const real_t step_size = 1e-10,
                         const real_t threshold = 1e-2) {
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

    // Calculate central finite difference
    fdiff.block(0, i, 2, 1) = (e_fd - e_bd) / (2 * step_size);
  }

  return check_jacobian("J_point", fdiff, J_point, threshold, true);
}

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
