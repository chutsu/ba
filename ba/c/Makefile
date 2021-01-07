MKFPATH:=$(abspath $(lastword $(MAKEFILE_LIST)))
MKFDIR:=$(dir $(MKFPATH))

BLD_DIR=build
BIN_DIR=bin

CC=gcc
CFLAGS= -O2 -Wall -I$(INC_DIR) -pedantic
LIBS=-L$(BLD_DIR) \
	-lba \
	-lblas \
	-llapack \
	-lpthread \
	-lm
OBJS=$(BLD_DIR)/util.o

LIBBA=$(BLD_DIR)/libba.a
BA_SOLVER=$(BIN_DIR)/ba_solver
TEST_BA=$(BIN_DIR)/test_ba

default: dirs $(OBJS) $(LIBBA) $(BA_SOLVER)
.PHONY: dirs

dirs:
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(BLD_DIR)

clean:
	@rm -rf $(BIN_DIR)
	@rm -rf $(BLD_DIR)

$(BLD_DIR)/%.o : %.c %.h
	@echo "CC [$<]"; \
	$(CC) $(CFLAGS) -c $< -o $@

$(LIBBA): $(OBJS)
	@echo "AR [libba.a]"
	@ar rvs $@ $^ 

# $(TEST_BA): test_ba.c ba.h
# 	@echo "TEST [test_ba]"
# 	$(CC) $^ -o $@ $(LIBS) $(CFLAGS)

$(BA_SOLVER): ba_solver.c $(OBJS)
	@echo "EXE [ba_solver]"
	@$(CC) $^ -o $@ $(LIBS) $(CFLAGS)
	@cd $(BIN_DIR); ln -sf $(MKFDIR)/../../scripts/sim_plot.m .
	@cd $(BIN_DIR); ln -sf $(MKFDIR)/../data_gnd .
	@cd $(BIN_DIR); ln -sf $(MKFDIR)/../data_noisy .
