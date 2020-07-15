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
	@make -s -C src
	@cp -R data bin
	@echo "Finished building! :)"
	@echo ""

unittest: ba
	@cd bin && ./test_ba

run: ba
	@cd bin && ./ba_solver ./data

clean:
	@echo "Cleaning repo..."
	@rm -rf bin
	@rm -rf build
	@echo "Done!"
