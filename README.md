# public_momapf

C++ Implementation of Multi-Objective Multi-Agent Path Finding Algorithms.

The code is distributed for academic and non-commercial use.

This project is still under active development.

## Requirements

* We use CMake (3.16.3) and Make (4.2.1) to compile the code. Lower or higher version may also work.

## Project Structure

* `README.md` - This file
* `cpp/source/` - Contains the path planning source code
* `cpp/test/` - Contains example code for MO-CBS algorithms
* `cpp/include/` - Contains header files

## Instructions

* Clone this repo
* Compile this repo
  * `mkdir build`
  * `cd build`
  * `cmake ..` (You can specify the build type you like by adding additional args)
  * `make`
* Run example in `./test_mocbs `
* This example shows how to use the code.

## References

* [1] Subdimensional Expansion for Multi-objective Multi-agent Path Finding\
	**Zhongqiang Ren**, Sivakumar Rathinam, and Howie Choset.\
	<i>IEEE Robotics and Automation Letters (RA-L)</i>, 2021.\
[[Bibtex](https://wonderren.github.io/files/bibtex_ren21momstar.txt)]
[[Paper](https://ieeexplore.ieee.org/document/9484849)]
[[Arxiv](https://arxiv.org/pdf/2102.01353.pdf)]
[[Talk](https://youtu.be/pfeBNvOqzvE)]

* [2] Multi-objective Conflict-based Search for Multi-agent Path Finding\
	**Zhongqiang Ren**, Sivakumar Rathinam, and Howie Choset.\
	<i>IEEE International Conference on Robotics and Automation (ICRA)</i>, 2021.\
[[Bibtex](https://wonderren.github.io/files/bibtex_ren21mocbs.txt)]
[[Paper](https://ieeexplore.ieee.org/document/9560985)]
[[Arxiv](https://arxiv.org/pdf/2101.03805.pdf)]
[[Talk](https://youtu.be/KI-BVhsjg0I)]

* [3] Multi-objective Conflict-based Search Using Safe-interval Path Planning\
  **Zhongqiang Ren**, Sivakumar Rathinam, Maxim Likhachev and Howie Choset.\
  <i>arXiv</i>\
[[Bibtex](https://wonderren.github.io/files/bibtex_ren22mosipp.txt)]
[[Arxiv](https://arxiv.org/pdf/2108.00745.pdf)]
