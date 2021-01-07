MKFPATH:=$(abspath $(lastword $(MAKEFILE_LIST)))
MKFDIR:=$(dir $(MKFPATH))

BLD_DIR=$(MKFDIR)/build
BIN_DIR=$(MKFDIR)/bin

CXX=g++
CXXFLAGS=-std=c++11 -Wall -O2 -I/usr/include/eigen3
LIBS=-L$(BLD_DIR) -lba
OBJS=$(BLD_DIR)/util.o $(BLD_DIR)/ba.o

LIBBA=$(BLD_DIR)/libba.a
BA_SOLVER=$(BIN_DIR)/ba_solver
TEST_BA=$(BIN_DIR)/test_ba

default: dirs $(LIBBA) $(TEST_BA) $(BA_SOLVER)
.PHONY: dirs

dirs:
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(BLD_DIR)

clean:
	@rm -rf $(BIN_DIR)
	@rm -rf $(BLD_DIR)

$(BLD_DIR)/%.o: %.cpp %.hpp
	@echo "CXX [$<]"
	@$(CXX) $(CXXFLAGS) -c $< -o $@

$(LIBBA): $(OBJS)
	@echo "AR [libba.a]"
	@ar rvs $@ $^

$(TEST_BA): test_ba.cpp ba.cpp
	@echo "TEST [test_ba]"
	$(CXX) $^ -o $@ $(LIBS) $(CXXFLAGS)

$(BA_SOLVER): ba_solver.cpp $(OBJS)
	@echo "EXE [ba_solver]"
	@$(CXX) $^ -o $@ $(LIBS) $(CXXFLAGS)
	@cd $(BIN_DIR); ln -sf $(MKFDIR)/../../scripts/sim_plot.m .
	@cd $(BIN_DIR); ln -sf $(MKFDIR)/../data_gnd .
	@cd $(BIN_DIR); ln -sf $(MKFDIR)/../data_noisy .
