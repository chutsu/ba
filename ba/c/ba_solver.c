#include "ba.h"

void print_usage() {
  printf("Usage: ba_solver <data>\n");
  printf("Example: ba_solver ./data\n");
}

int main(int argc, char **argv) {
  /* const char *save_path = "./data_est"; */
  if (argc != 2) {
    print_usage();
    return -1;
  }

  ba_data_t *data = ba_load_data(argv[1]);
  printf("Solving BA problem:\n");
  struct timespec t_start = tic();
  ba_solve(data);
  printf("total time taken: %.4fs\n", toc(&t_start));
  printf("nb_frames: %d\n", data->nb_frames);
  printf("nb_points: %d\n", data->nb_points);
  ba_data_free(data);

  return 0;
}
