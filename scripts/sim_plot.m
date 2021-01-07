#!/usr/bin/env octave-cli
graphics_toolkit("fltk");
save_figs = false;
show_figs = true;
data_gnd_dir = "./data_gnd";  % Ground truth data
data_noisy_dir = "./data_noisy";  % Noisy data
data_est_dir = "./data_est";  % Estimated data

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

function cam_K = load_cam_K(data_dir)
  cam_K = csvread(strcat(data_dir, "/camera.csv"), 1, 0);
endfunction

function cam_poses = load_cam_poses(data_dir)
  csv = csvread(strcat(data_dir, "/camera_poses.csv"), 1, 0);
  cam_poses = {};
  for i = 1:rows(csv)
    cam_poses{i}.q_WC = csv(i, 1:4)';
    cam_poses{i}.r_WC = csv(i, 5:7)';
  endfor
endfunction

function keypoints = load_keypoints(data_dir)
  csv = csvread(strcat(data_dir, "/keypoints.csv"), 1, 0);
  keypoints = {};
  for i = 1:rows(csv)
    keypoints.frame{i} = {};
    nb_keypoints = csv(i, 1);
    for j = 1:nb_keypoints
      keypoints.frame{i}{j} = csv(i, 1 + j);
    endfor
  endfor
endfunction

function point_ids = load_point_ids(data_dir)
  csv = csvread(strcat(data_dir, "/point_ids.csv"), 1, 0);
  point_ids = {};
  for i = 1:rows(csv)
    point_ids.frame{i} = {};
    nb_point_ids = csv(i, 1);
    for j = 1:nb_point_ids
      point_ids.frame{i}{j} = csv(i, 1 + j);
    endfor
  endfor
endfunction

function points = load_points(data_dir)
  points = csvread(strcat(data_dir, "/points.csv"), 1, 0);
endfunction

function T_WT = load_target_pose(data_dir)
  csv = csvread(strcat(data_dir, "/target_pose.csv"), 1, 0);
  q_WT = csv(1, 1:4)';
  r_WT = csv(1, 5:7)';
  T_WT = tf(q_WT, r_WT);
endfunction

function data = load_data(data_dir)
  data = {};
  data.cam_K = load_cam_K(data_dir);
  data.cam_poses = load_cam_poses(data_dir);
  data.keypoints = load_keypoints(data_dir);
  data.point_ids = load_point_ids(data_dir);
  data.points = load_points(data_dir);
  data.T_WT = load_target_pose(data_dir);
endfunction

function draw_frame(T_WB, scale=1.1)
  r_WB = tf_trans(T_WB);
  origin = r_WB;

  x_axis = T_WB * homogeneous(scale * [1; 0; 0]);
  y_axis = T_WB * homogeneous(scale * [0; 1; 0]);
  z_axis = T_WB * homogeneous(scale * [0; 0; 1]);

  % Draw x-axis
  plot3([origin(1), x_axis(1)], ...
        [origin(2), x_axis(2)], ...
        [origin(3), x_axis(3)], 'r',
        "linewidth", 5)

  % Draw y-axis
  plot3([origin(1), y_axis(1)], ...
        [origin(2), y_axis(2)], ...
        [origin(3), y_axis(3)], 'g',
        "linewidth", 5)

  % Draw z-axis
  plot3([origin(1), z_axis(1)], ...
        [origin(2), z_axis(2)], ...
        [origin(3), z_axis(3)], 'b',
        "linewidth", 5)
endfunction

function draw_camera(T_WC, scale=0.05, style="b-")
  fov = deg2rad(60.0);

  % Form the camera fov frame
  fov_hwidth = scale;
  fov_corners = zeros(3, 4);
  fov_corners(1:3, 1) = [-fov_hwidth; fov_hwidth; 0.0];  % Bottom left
  fov_corners(1:3, 2) = [-fov_hwidth; -fov_hwidth; 0.0]; % Top left
  fov_corners(1:3, 3) = [fov_hwidth; -fov_hwidth; 0.0];  % Top right
  fov_corners(1:3, 4) = [fov_hwidth; fov_hwidth; 0.0];   % Bottom right

  % Calculate the distance from camera origin to fov frame given fov
  dist = fov_hwidth / tan(fov / 2.0);
  fov_corners(3, :) = dist;

  % Transform fov_corners to world frame
  fov_corners = T_WC * [fov_corners; ones(1, 4)];
  fov_corners = fov_corners(1:3, :);

  % Transform camera_origin to world frame
  cam_origin = [0; 0; 0];
  cam_origin = T_WC * [cam_origin; 1.0];
  cam_origin = cam_origin(1:3, :);

  % Draw fov frame
  frame_x = [fov_corners(1, :), fov_corners(1, 1)];
  frame_y = [fov_corners(2, :), fov_corners(2, 1)];
  frame_z = [fov_corners(3, :), fov_corners(3, 1)];
  plot3(frame_x, frame_y, frame_z, style);

  % Draw from camera origin to fov frame
  for i = 1:4
    x = [cam_origin(1), fov_corners(1, i)];
    y = [cam_origin(2), fov_corners(2, i)];
    z = [cam_origin(3), fov_corners(3, i)];
    plot3(x, y, z, style);
  endfor
endfunction

function plot_data(data)
  figure();
  clf();
  hold on;

  % Draw camera poses
  for i = 1:length(data.cam_poses)
    q_WC = data.cam_poses{i}.q_WC;
    r_WC = data.cam_poses{i}.r_WC;
    T_WC = tf(q_WC, r_WC);

    draw_camera(T_WC);
    draw_frame(T_WC, 0.05);
  endfor

  % 3D landmark points
  scatter3(data.points(:, 1),
             data.points(:, 2),
             data.points(:, 3), 'r');

  xlabel("x [m]");
  ylabel("y [m]");
  zlabel("z [m]");
  view(3);
  axis 'equal';
  ginput;
endfunction

function plot_compare_data(title_name, data0, data1, save_figs)
  figure();
  clf();
  hold on;

  for i = 1:length(data0.cam_poses)
    q_WC = data0.cam_poses{i}.q_WC;
    r_WC = data0.cam_poses{i}.r_WC;
    T_WC = tf(q_WC, r_WC);

    draw_camera(T_WC, 0.05, 'r-');
    % draw_frame(T_WC, 0.05);
  endfor

  for i = 1:length(data1.cam_poses)
    q_WC = data1.cam_poses{i}.q_WC;
    r_WC = data1.cam_poses{i}.r_WC;
    T_WC = tf(q_WC, r_WC);

    draw_camera(T_WC, 0.05, 'b-');
    % draw_frame(T_WC, 0.05);
  endfor

  scatter3(data0.points(:, 1),
           data0.points(:, 2),
           data0.points(:, 3), 'r');

  scatter3(data1.points(:, 1),
           data1.points(:, 2),
           data1.points(:, 3), 'b');

  % Plot settings
  if save_figs
    title(title_name, "fontsize", 40);
  else
    title(title_name, "fontsize", 20);
  end
  xlabel("x [m]");
  ylabel("y [m]");
  zlabel("z [m]");
  view(3);
  axis 'equal';
endfunction

% Main
data_gnd = load_data(data_gnd_dir);
data_noisy = load_data(data_noisy_dir);
data_est = load_data(data_est_dir);

plot_compare_data("Before Optimization", data_gnd, data_noisy, save_figs);
if save_figs
  print('-dpng', '-S1200,1200', 'ba_before.png')
endif

plot_compare_data("After Optimization", data_gnd, data_est, save_figs);
if save_figs
  print('-dpng', '-S1200,1200', 'ba_after.png')
endif

if show_figs
  ginput();
endif
