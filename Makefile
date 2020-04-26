ba: ba.cpp util.cpp
	@echo "g++ [$<]"
	@g++ $^ -o ba -I/usr/include/eigen3 -lyaml-cpp -O2

clean:
	rm ba
