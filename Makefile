define usage
[MAKE TARGETS]:
  deps:
    Install dependencies

  ba:
    Build repo

  unittest:
    Build and run unittest

  run:
    Run bundle adjustment solver

  plot:
    Plot results

  clean:
    Clean build and bin directories
endef
export usage

.PHONY: ba deps unittest run clean

default:
	@echo "$$usage"

deps:
	@echo "Installing dependencies ..."
	@sudo apt-get update
	@sudo apt-get install -qqq libyaml-cpp-dev libeigen3-dev octave

ba:
	@make -s -C ba/c
	@make -s -C ba/cpp

unittest: ba
	@cd bin && ./test_ba

run: ba
	@cd ba/cpp/bin && ./ba_solver ./data_noisy

plot: ba run
	@cd ba/cpp/bin && octave-cli sim_plot.m

runc: ba
	@cd ba/c/bin && ./ba_solver ./data_noisy

clean:
	@echo "Cleaning repo..."
	@make -s -C ba/c clean
	@make -s -C ba/cpp clean
	@echo "Done!"
