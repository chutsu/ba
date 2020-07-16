BIN_DIR=bin
BLD_DIR=build

define usage
[MAKE TARGETS]:
  ba:
    Build repo

  unittest:
    Build and run unittest

  run:
    Run bundle adjustment solver

  clean:
    Clean build and bin directories
endef
export usage

.PHONY: ba

default:
	@echo "$$usage"

dirs:
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(BLD_DIR)

ba: dirs
	@make -s -C ba
	@echo "Finished building! :)"

unittest: ba
	@cd bin && ./test_ba

run: ba
	@cd bin && ./ba_solver ./data_noisy
	@cd bin && octave-cli ./sim_plot.m ./data_noisy

clean:
	@echo "Cleaning repo..."
	@rm -rf bin
	@rm -rf build
	@echo "Done!"
