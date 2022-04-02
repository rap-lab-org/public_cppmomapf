# Public C++ MOMAPF

This repository provides C++ Implementation of several Multi-Objective Multi-Agent Path Finding algorithms, including MOM* [1] and MO-CBS [2]. The code is distributed for academic and non-commercial use, and this project is still under active development. 

Note: An older preliminary Python implementation of the algorithms can be found [there](https://github.com/wonderren/public_pymomapf), which is not maintained any more. This C++ package is recommended for reference and further development.

## Requirements

* We use CMake (3.16.3) and Make (4.2.1) on Ubuntu 20.04 to compile the code. Lower or higher version may also work. 
* We use C++11. Lower or higher version of C++ may also work.
* We massively use C++ STL, which is very common in C++ programming nowadays. We do not use any third-party libraries, and this package should be straightfoward to compile by following the instructions below.

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
