#include "ba.hpp"
#include "util.hpp"

void print_usage() {
  printf("Usage: ba_solver <data>\n");
  printf("Example: ba_solver ./data\n");
}

int main(int argc, char **argv) {
  if (argc != 2) {
    print_usage();
    return -1;
  }

  struct timespec t_start = tic();
  ba_data_t data{std::string{argv[1]}};
  printf("Solving BA problem:\n");
  ba_solve(data);
  printf("time taken: %fs\n", toc(&t_start));
  printf("nb_frames: %d\n", data.nb_frames);
  printf("nb_points: %d\n", data.nb_points);
  printf("\n");

  return 0;
}
