# Public C++ MOMAPF

This repository provides C++ Implementation of several Multi-Objective Multi-Agent Path Finding algorithms, including MOM* [1] and MO-CBS [2]. This project is still under active development. 

Note: An older preliminary Python implementation of the algorithms can be found [there](https://github.com/wonderren/public_pymomapf), which is not maintained any more. This C++ package is recommended for reference and further development.


<p align="center">
<img src="https://github.com/wonderren/wonderren.github.io/blob/master/images/fig_mocbs_construction.png" alt="" hspace="15" style=" border: #FFFFFF 2px none;">
</p>

(Fig 1: A toy example about planning conflict-free joint paths for multiple agents to transport materials in a construction site while optimizing both path risk and arrival time. The black cells are semi-constructed architectures and there is a risk of collision or falling items in the neighborhood of each black cell. The darkness of each grey cell indicates the risk cost. (a) shows the Pareto-optimal front (the cost vectors corresponding to joint paths). (b), (c) and (d) show three Pareto-optimal solution joint paths corresponding to the red, green and orange solution in (a) respectively. In (b), (c) and (d), the colored dotted paths show the individual paths that constitute the corresponding joint path, while the black dotted paths show the individual paths in other Pareto-optimal solutions. For solution S1, all agents take shortcut and go through risky zones while for solution S2, all agents are being conservative and go through safe zones. The solution S3 balances the two objectives. Finding and visualizing a Pareto-optimal set of solutions can potentially help the human decision maker to understand the underlying trade-off between conflicting objectives and thus make more informed decisions.)

The code is distributed for academic and non-commercial use.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

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

Please cite the corresponding papers if you find anything in this repository helpful.

* [1] Subdimensional Expansion for Multi-objective Multi-agent Path Finding\
	**Zhongqiang Ren**, Sivakumar Rathinam, and Howie Choset.\
	<i>IEEE Robotics and Automation Letters (RA-L)</i>, 2021.\
[[Bibtex](https://wonderren.github.io/files/bibtex_ren21momstar.txt)]
[[Paper](https://wonderren.github.io/files/ren21-MOMstar_RAL_IROS.pdf)]
[[Talk](https://youtu.be/pfeBNvOqzvE)]

* [2] A Conflict-Based Search Framework for Multi-Objective Multi-Agent Path Finding\
	**Zhongqiang Ren**, Sivakumar Rathinam, and Howie Choset.\
	<i>IEEE Transactions on Automation Science and Engineering (T-ASE)</i>, 2022.\
[[Bibtex](https://wonderren.github.io/files/bibtex_ren21mocbs_tase.txt)]
[[Paper](https://wonderren.github.io/files/ren22_mocbs_tase_final.pdf)]
[[Talk](https://youtu.be/KI-BVhsjg0I)]
