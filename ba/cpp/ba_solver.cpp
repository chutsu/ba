#include "ba.hpp"

void print_usage() {
  printf("Usage: ba_solver <data>\n");
  printf("Example: ba_solver ./data\n");
}

int main(int argc, char **argv) {
  std::string save_path = "./data_est";
  if (argc != 2) {
    print_usage();
    return -1;
  }

  struct timespec t_start = tic();
  ba_data_t data{std::string{argv[1]}};
  printf("Solving BA problem:\n");
  ba_solve(data);
  printf("total time taken: %.4fs\n", toc(&t_start));
  printf("nb_frames: %d\n", data.nb_frames);
  printf("nb_points: %d\n", data.nb_points);
  printf("\n");
  ba_save(data, save_path);

  return 0;
}
