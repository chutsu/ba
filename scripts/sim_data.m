#!/usr/bin/env octave-cli
data_gnd_path = "./data_gnd";
data_noisy_path = "./data_noisy";

function q = rot2quat(R)
  m00 = R(1, 1);
  m01 = R(1, 2);
  m02 = R(1, 3);

  m10 = R(2, 1);
  m11 = R(2, 2);
  m12 = R(2, 3);

  m20 = R(3, 1);
  m21 = R(3, 2);
  m22 = R(3, 3);

  tr = m00 + m11 + m22;

  if (tr > 0)
    S = sqrt(tr+1.0) * 2; % S=4*qw
    qw = 0.25 * S;
    qx = (m21 - m12) / S;
    qy = (m02 - m20) / S;
    qz = (m10 - m01) / S;
  elseif ((m00 > m11) && (m00 > m22))
    S = sqrt(1.0 + m00 - m11 - m22) * 2; % S=4*qx
    qw = (m21 - m12) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S;
    qz = (m02 + m20) / S;
  elseif (m11 > m22)
    S = sqrt(1.0 + m11 - m00 - m22) * 2; % S=4*qy
    qw = (m02 - m20) / S;
    qx = (m01 + m10) / S;
    qy = 0.25 * S;
    qz = (m12 + m21) / S;
  else
    S = sqrt(1.0 + m22 - m00 - m11) * 2; % S=4*qz
    qw = (m10 - m01) / S;
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  endif

  q = [qw; qx; qy; qz];
endfunction

function R = quat2rot(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;
  qw2 = qw**2;

  % Homogeneous form
  R11 = qw2 + qx2 - qy2 - qz2;
  R12 = 2 * (qx * qy - qw * qz);
  R13 = 2 * (qx * qz + qw * qy);

  R21 = 2 * (qx * qy + qw * qz);
  R22 = qw2 - qx2 + qy2 - qz2;
  R23 = 2 * (qy * qz - qw * qx);

  R31 = 2 * (qx * qz - qw * qy);
  R32 = 2 * (qy * qz + qw * qx);
  R33 = qw2 - qx2 - qy2 + qz2;

  R = [R11, R12, R13; R21, R22, R23; R31, R32, R33;];
endfunction

function euler = quat2euler(q)
  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  qw2 = qw**2;
  qx2 = qx**2;
  qy2 = qy**2;
  qz2 = qz**2;

  t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  t2 = asin(2 * (qy * qw - qx * qz));
  t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));
  euler = [t1; t2; t3];
endfunction

function R = euler321(rpy)
  phi = rpy(1);
  theta = rpy(2);
  psi = rpy(3);

  R11 = cos(psi) * cos(theta);
  R21 = sin(psi) * cos(theta);
  R31 = -sin(theta);

  R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  R32 = cos(theta) * sin(phi);

  R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  R33 = cos(theta) * cos(phi);

  R = [R11, R12, R13; R21, R22, R23; R31, R32, R33;];
endfunction

function dq = quat_delta(dalpha)
  half_norm = 0.5 * norm(dalpha);
  vector = sinc(half_norm) * 0.5 * dalpha;
  scalar = cos(half_norm);
  dq = [scalar; vector];
endfunction

function result = quat_mul(p, q)
  % p_w = p(1);
  % p_x = p(2);
  % p_y = p(3);
  % p_z = p(4);
  %
  % q_w = q(1);
  % q_x = q(2);
  % q_y = q(3);
  % q_z = q(4);
  %
  % w = p_w * q_w - p_x * q_x - p_y * q_y - p_z * q_z;
  % x = p_w * q_x + q_x * p_w + p_y * q_z - p_z * q_y;
  % y = p_w * q_y - p_y * q_w + p_z * q_x + p_x * q_z;
  % z = p_w * q_z + p_z * q_w - p_x * q_y + p_y * q_x;

  % result = [w; x; y; z];

  p_w = p(1);
  q_w = q(1);
  p_v = [p(2); p(3); p(4)];
  q_v = [q(2); q(3); q(4)];

  result = [p_w * q_w - p_v' * q_v;
            p_w * q_v + q_w * p_v + cross(p_v, q_v);];
endfunction

function r = quatmul(p, q)
  r = quatlmul(p, q);
endfunction

function r = quatrmul(p, q)
  assert(size(p) == [4, 1]);
  assert(size(q) == [4, 1]);

  qw = q(1);
  qx = q(2);
  qy = q(3);
  qz = q(4);

  rprod = [
    qw, -qx, -qy, -qz;
    qx, qw, qz, -qy;
    qy, -qz, qw, qx;
    qz, qy, -qx, qw;
  ];

  r = rprod * p;
endfunction

function T = tf(rot, trans)
  assert(size(rot) == [3, 3] || size(rot) == [4, 1]);
  assert(size(trans) == [3, 1]);

  C = rot;
  if size(rot) == [4, 1]
    C = quat2rot(rot);
  endif

  T = eye(4, 4);
  T(1:3, 1:3) = C;
  T(1:3, 4) = trans;
endfunction

function C = tf_rot(tf)
  C = tf(1:3, 1:3);
endfunction

function r = tf_trans(T)
  r = T(1:3, 4);
endfunction

function hp = homogeneous(p)
  hp = [p; ones(1, columns(p))];
endfunction

function p = dehomogeneous(hp)
  p = hp(1:3, :);
endfunction

function y = normalize(x)
  y = x / norm(x);
endfunction

function retval = randf(bounds, sz=1)
  val_min = bounds(1);
  val_max = bounds(2);
  retval = (val_max - val_min) * rand(sz, 1) + val_min;
endfunction

function T_target_camera = lookat(cam_pos, target, up_axis=[0.0; -1.0; 0.0])
  assert(size(cam_pos) == [3, 1]);
  assert(size(target) == [3, 1]);
  assert(size(up_axis) == [3, 1]);

  % Note: If we were using OpenGL the cam_dir would be the opposite direction,
  % since in OpenGL the camera forward is -z. In robotics however our camera is
  % +z forward.
  cam_dir = normalize((target - cam_pos));
  cam_right = normalize(cross(up_axis, cam_dir));
  cam_up = cross(cam_dir, cam_right);

  A = [cam_right(1), cam_right(2), cam_right(3), 0.0;
       cam_up(1), cam_up(2), cam_up(3), 0.0;
       cam_dir(1), cam_dir(2), cam_dir(3), 0.0;
       0.0, 0.0, 0.0, 1.0;];

  B = [1.0, 0.0, 0.0, -cam_pos(1);
       0.0, 1.0, 0.0, -cam_pos(2);
       0.0, 0.0, 1.0, -cam_pos(3);
       0.0, 0.0, 0.0, 1.0;];

  T_camera_target = A * B;
  T_target_camera = inv(T_camera_target);
endfunction

function fx = focal_length(image_width, fov)
  fx = (image_width / 2.0) / tan(deg2rad(fov / 2.0));
endfunction

function K = pinhole_K(intrinsics)
  fx = intrinsics(1);
  fy = intrinsics(2);
  cx = intrinsics(3);
  cy = intrinsics(4);

  K = eye(3);
  K(1, 1) = fx;
  K(2, 2) = fy;
  K(1, 3) = cx;
  K(2, 3) = cy;
endfunction

function camera = camera_init(resolution, fov, D=zeros(4, 1))
  fx = focal_length(resolution(1), fov);
  fy = focal_length(resolution(2), fov);
  cx = resolution(1) / 2.0;
  cy = resolution(2) / 2.0;
  intrinsics = [fx, fy, cx, cy];

  camera.resolution = resolution;
  camera.K = pinhole_K(intrinsics);
  camera.D = D;
endfunction

function [z, point_ids] = camera_measurements(K, resolution, T_WC, points_W)
  assert(size(K) == [3, 3]);
  assert(length(resolution) == 2);
  assert(size(T_WC) == [4, 4]);
  assert(rows(points_W) == 3);
  assert(columns(points_W) > 0);

  # Form projection matrix
  T_CW = inv(T_WC);
  R_CW = T_CW(1:3, 1:3);
  t_CW = T_CW(1:3, 4);
  P = K * [R_CW, t_CW];

  # Setup
  image_width = resolution(1);
  image_height = resolution(2);
  nb_points = columns(points_W);

  # Check points
  z = [];
  point_ids = [];

  for i = 1:nb_points
    # Project point to image plane
    hp_W = homogeneous(points_W(:, i));
    x = P * hp_W;

    # Check to see if point is infront of camera
    if x(3) < 1e-4
      continue;
    endif

    # Normalize projected ray
    x(1) = x(1) / x(3);
    x(2) = x(2) / x(3);
    x(3) = x(3) / x(3);
    z_hat = [x(1); x(2)];

    # Check to see if ray is within image plane
    x_ok = (x(1) < image_width) && (x(1) > 0.0);
    y_ok = (x(2) < image_height) && (x(2) > 0.0);
    if x_ok && y_ok
      z = [z, z_hat];
      point_ids = [point_ids, i];
    endif
  endfor
  assert(columns(z) == length(point_ids));
endfunction

function [poses, calib_center] = calib_generate_random_poses(calib_target,
                                                             nb_poses)
  calib_width = (calib_target.nb_rows - 1.0) * calib_target.tag_size;
  calib_height = (calib_target.nb_cols - 1.0) * calib_target.tag_size;
  calib_center = [calib_width / 2.0; calib_height / 2.0; 0.0];

  % Settings
  angle_range = [-20.0, 20.0];
  x_range = [-0.5, 0.5];
  y_range = [-0.5, 0.5];
  z_range = [0.5, 0.7];

  % For each position create a camera pose that "looks at" the AprilGrid
  % center in the target frame, T_TC.
  poses = {};
  pose_idx = 1;
  for i = 1:nb_poses
    % Generate random pose
    x = unifrnd(x_range(1), x_range(2));
    y = unifrnd(y_range(1), y_range(2));
    z = unifrnd(z_range(1), z_range(2));
    r_TC = calib_center + [x; y; z] ;
    T_TC = lookat(r_TC, calib_center);

    % Perturb the pose a little so it doesn't look at the center directly
    p = randf(angle_range);
    q = randf(angle_range);
    r = randf(angle_range);
    C_perturb = euler321(deg2rad([p, q, r]));
    r_perturb = zeros(3, 1);
    T_perturb = tf(C_perturb, r_perturb);

    poses{pose_idx} = T_perturb * T_TC;
    pose_idx++;
  endfor
endfunction

function calib_target = calib_target_init(nb_rows=6, nb_cols=7, tag_size=0.2)
  # Create calib_target grid
  nb_corners = nb_rows * nb_cols;
  object_points = zeros(3, nb_corners);
  idx = 1;
  for i = 1:nb_rows
    for j = 1:nb_cols
      object_points(1:2, idx) = [j - 1; i - 1];
      idx += 1;
    endfor
  endfor

  # Chessboard struct
  calib_target.nb_rows = nb_rows;
  calib_target.nb_cols = nb_cols;
  calib_target.nb_corners = nb_corners;
  calib_target.tag_size = tag_size;
  calib_target.width = calib_target.nb_cols * calib_target.tag_size;
  calib_target.height = calib_target.nb_rows * calib_target.tag_size;
  calib_target.center = [((nb_cols - 1.0) / 2.0) * tag_size,
                         ((nb_rows - 1.0) / 2.0) * tag_size];
  calib_target.object_points = tag_size * object_points;
endfunction

function data = calib_sim(calib_target, T_WT, camera, nb_poses, debug=false)
  % Generate camera poses
  [poses, calib_center] = calib_generate_random_poses(calib_target, nb_poses);
  nb_poses = length(poses);

  % Transform calibration grid points from target to world frame
  p_data = [];
  hp_T = homogeneous(calib_target.object_points);
  hp_W = T_WT * hp_T;
  p_data = dehomogeneous(hp_W);
  assert(rows(calib_target.object_points) == 3);
  assert(rows(p_data) == 3);

  % Decompose target pose to quaternion and translation vector
  C_WT = tf_rot(T_WT);
  r_WT = tf_trans(T_WT);
  q_WT = rot2quat(C_WT);
  assert(norm(quat2rot(q_WT) - C_WT) < 1e-4);

  % Simulation calibration data
  point_ids_data = {};
  z_data = {};
  q_WC = {};
  r_WC = {};
  cam_poses = {};

  for i = 1:nb_poses
    % Transform camera pose in target frame to world frame
    T_TC = poses{i};
    T_WC = T_WT * T_TC;
    cam_poses{i} = T_WC;

    % Decompose camera pose to quaternion and translation vector
    C_WC = tf_rot(T_WC);
    q_WC{i} = rot2quat(C_WC);
    r_WC{i} = tf_trans(T_WC);
    assert(norm(quat2rot(q_WC{i}) - C_WC) < 1e-4);

    % Project world points to camera
    K = camera.K;
    image_size = camera.resolution;
    [z, point_ids] = camera_measurements(K, image_size, T_WC, p_data);

    z_data{i} = z;
    point_ids_data{i} = point_ids;
  endfor

  % Form simulation data struct
  % -- Time, camera and calib_target data
  data.time = 1:nb_poses;
  data.camera = camera;
  data.target = calib_target;
  % -- Calibration target
  data.q_WT = q_WT;
  data.r_WT = r_WT;
  % -- Camera poses
  data.q_WC = q_WC;
  data.r_WC = r_WC;
  % -- Image measurements and corresponding landmark points
  data.z_data = z_data;
  data.point_ids_data = point_ids_data;
  data.p_data = p_data;

  % Visualize
  if debug
    figure()
    hold on;
    for i = 1:nb_poses
      calib_target_draw(calib_target, T_WT);
      T_WC = cam_poses{i};
      draw_camera(T_WC);
      draw_frame(T_WC, 0.05);
    endfor
    view(3);
    axis("equal");
    xlabel("x [m]");
    ylabel("y [m]");
    zlabel("z [m]");
    ginput();
  endif
endfunction

function data_noisy = calib_data_add_noise(data)
  nb_poses = length(data.time);

  % Add noise to camera position and rotation
  for i = 1:nb_poses
    % Position
    % data.r_WC{i} += normrnd([0; 0; 0], 1e-2);
    data.r_WC{i} += normrnd([0; 0; 0], 1e-1);

    % Rotation
    dq = quat_delta(normrnd([0; 0; 0], 1e-2));
    q_WC = data.q_WC{i};
    data.q_WC{i} = quat_mul(dq, q_WC);
  endfor

  % Add noise to point data
  % data.p_data += normrnd(zeros(3, length(data.p_data)), 1e-2);

  data_noisy = data;
endfunction

function save_dataset(save_path, data)
  % Create directory to save dataset
  mkdir(save_path);

  % -- Save camera matrix
  cam_file = fopen(strcat(save_path, "camera.csv"), "w");
  K = data.camera.K;
  fprintf(cam_file, "#camera_K\n");
  fprintf(cam_file, "%f,%f,%f\n", K(1, 1), K(1, 2), K(1, 3));
  fprintf(cam_file, "%f,%f,%f\n", K(2, 1), K(2, 2), K(2, 3));
  fprintf(cam_file, "%f,%f,%f\n", K(3, 1), K(3, 2), K(3, 3));
  fclose(cam_file);

  % -- Save camera poses
  cam_poses_file = fopen(strcat(save_path, "camera_poses.csv"), "w");
  fprintf(cam_poses_file, "#qw,qx,qy,qz,rx,ry,rz\n");
  for k = 1:length(data.q_WC)
    q = data.q_WC{k};
    r = data.r_WC{k};
    fprintf(cam_poses_file, "%f,%f,%f,%f,", q(1), q(2), q(3), q(4));
    fprintf(cam_poses_file, "%f,%f,%f", r(1), r(2), r(3));
    fprintf(cam_poses_file, "\n");
  endfor
  fclose(cam_poses_file);

  % -- Save target pose
  target_pose_file = fopen(strcat(save_path, "target_pose.csv"), "w");
  fprintf(target_pose_file, "#qw,qx,qy,qz,rx,ry,rz\n");
  q = data.q_WT;
  r = data.r_WT;
  fprintf(target_pose_file, "%f,%f,%f,%f,", q(1), q(2), q(3), q(4));
  fprintf(target_pose_file, "%f,%f,%f", r(1), r(2), r(3));
  fprintf(target_pose_file, "\n");
  fclose(target_pose_file);

  % -- Save measured keypoints
  keypoints_file = fopen(strcat(save_path, "keypoints.csv"), "w");
  fprintf(keypoints_file, "#size,keypoints\n");
  for i = 1:length(data.z_data)
    z_data = data.z_data{i};
    fprintf(keypoints_file, "%d,", length(z_data) * 2);
    for j = 1:length(z_data)
      fprintf(keypoints_file, "%f,%f", z_data(1, j), z_data(2, j));
      if j != length(z_data)
        fprintf(keypoints_file, ",");
      end
    endfor
    fprintf(keypoints_file, "\n");
  endfor
  fclose(keypoints_file);

  % -- Save point ids
  point_ids_file = fopen(strcat(save_path, "point_ids.csv"), "w");
  fprintf(point_ids_file, "#size,point_ids\n");
  for i = 1:length(data.point_ids_data)
    id = data.point_ids_data{i};
    fprintf(point_ids_file, "%d,", length(id));
    for j = 1:length(id)
      fprintf(point_ids_file, "%d", id(j) - 1);  # to correct for 0-index
      if j != length(id)
        fprintf(point_ids_file, ",");
      end
    endfor
    fprintf(point_ids_file, "\n");
  endfor
  fclose(point_ids_file);

  % -- Save points
  points_file = fopen(strcat(save_path, "points.csv"), "w");
  fprintf(points_file, "#x,y,z\n");
  for i = 1:length(data.p_data)
    p = data.p_data(1:3, i);
    fprintf(points_file, "%f,%f,%f", p(1), p(2), p(3));
    if j != length(id)
      fprintf(points_file, ",");
    end
    fprintf(points_file, "\n");
  endfor
  fclose(points_file);
end

% Setup data
% -- Create calibration target
calib_target = calib_target_init(5, 5);
C_WT = euler321(deg2rad([90.0, 0.0, -90.0]));
r_WT = [1.0; 0.0; 0.0];
T_WT = tf(C_WT, r_WT);
% -- Create camera
res = [640; 480];
fov = 90.0;
camera = camera_init(res, fov);
% -- Create data
nb_poses = 20;
data_gnd = calib_sim(calib_target, T_WT, camera, nb_poses);
data_noisy = calib_data_add_noise(data_gnd);
% -- Save data
save_dataset("./data_gnd/", data_gnd);
save_dataset("./data_noisy/", data_noisy);
