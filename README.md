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

Example output:

```
Solving BA problem:
  - iter[0]   cost: 1.05e+06   time: 0.004s   rmse_reproj_error: 47.07px
  - iter[1]   cost: 3.01e+04   time: 0.004s   rmse_reproj_error: 7.97px
  - iter[2]   cost: 1.76e+02   time: 0.004s   rmse_reproj_error: 0.61px
  - iter[3]   cost: 1.84e-01   time: 0.004s   rmse_reproj_error: 0.02px
  - iter[4]   cost: 2.51e-04   time: 0.004s   rmse_reproj_error: 0.00px
  - iter[5]   cost: 2.57e-06   time: 0.004s   rmse_reproj_error: 0.00px
Done!
total time taken: 0.0257s
nb_frames: 20
nb_points: 25
```

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
