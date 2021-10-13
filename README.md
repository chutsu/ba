# ba

<a href="https://github.com/chutsu/ba/actions?query=ci">
  <img src="https://github.com/chutsu/ba/workflows/ci/badge.svg">
</a>

Minimal Bundle Adjustment Example:

<p align="center">
  <img src="imgs/example.png" alt="BA Demo" width="100%" />
</p>

where red represents the ground truth and blue represents the initial camera
poses and landmark points before and after bundle adjustment.

Example output on a Lenovo ThinkPad P15 Gen 1 with Intel Core i7:

Single precision mode:

```
Solving BA problem:
iter: [0]  lambda: 1.00e-04  cost: 3.0443e+04  dcost: -1.02e+06  rms reproj error: 11.35
iter: [1]  lambda: 1.00e-05  cost: 1.7417e+02  dcost: -3.03e+04  rms reproj error: 0.86
iter: [2]  lambda: 1.00e-06  cost: 9.8439e-02  dcost: -1.74e+02  rms reproj error: 0.02
iter: [3]  lambda: 1.00e-07  cost: 6.8691e-07  dcost: -9.84e-02  rms reproj error: 0.00
Done!
total time taken: 0.0239s
nb_frames: 20
nb_points: 25
```

Double precision mode:

```
Solving BA problem:
iter: [0]  lambda: 1.00e-04  cost: 3.0441e+04  dcost: -1.02e+06  rms reproj error: 11.35
iter: [1]  lambda: 1.00e-05  cost: 1.7375e+02  dcost: -3.03e+04  rms reproj error: 0.86
iter: [2]  lambda: 1.00e-06  cost: 9.7500e-02  dcost: -1.74e+02  rms reproj error: 0.02
iter: [3]  lambda: 1.00e-07  cost: 1.1059e-07  dcost: -9.75e-02  rms reproj error: 0.00
Done!
total time taken: 0.0322s
nb_frames: 20
nb_points: 25
```

An approximate ~34% speed up of single vs double precision.

## Dependencies

    LAPACKE (http://www.netlib.org/lapack/lapacke.html)
    Eigen3 (http://eigen.tuxfamily.org/)
    octave (https://www.gnu.org/software/octave/)

If youre in Ubuntu, you can obtain the dependencies via:

	sudo apt-get install -qqq \
		libblas-dev \
		liblapack-dev \
		liblapacke-dev \
		libeigen3-dev \
		octave


## Build and Run

    git clone https://github.com/chutsu/ba
    cd ba
    make deps
    make ba
    make run
    make plot


## Licence

```
Copyright (c) <2020> <Chris Choi>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
