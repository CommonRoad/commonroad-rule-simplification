## CommonRoad-Knowledge-Extraction: Extract Scenario-Specific Knowledge to Simplify Traffic Rules

### System Requirements

The software is written in Python 3.10 and C++-20, and was tested on Ubuntu 22.04.

### Building from Source

> **Note:** The build process automatically includes other internal repositories via Git.
> Thus, an SSH key in your LRZ GitLab account is required.
> See [here](https://docs.gitlab.com/ee/ssh/) for instructions on how to add an SSH key.

> **Note:** This project depends on the [`ltl_augmentation`](https://gitlab.lrz.de/cps/ltl-augmentation) Python package,
> which uses a Rust extension module.
> Currently, we do not provide a pre-built version of the `ltl_augmentation`, so you will need the Rust toolchain
> installed on your system to build the package.
> Please refer to the [Rust installation guide](https://www.rust-lang.org/tools/install) for further information.

#### Third-Party Dependencies

While most of these dependencies are added automatically during the build process, you can install them manually via
your package manager to speed up the build process.

**Manual installation recommended to speed up the build:**

- [Boost](https://www.boost.org/)

**Manual installation optional:**

- [Eigen3](https://eigen.tuxfamily.org/)
- [spdlog](https://github.com/gabime/spdlog)
- [nanobind](https://github.com/wjakob/nanobind)

**Optional dependencies:**

- [GTest](https://google.github.io/googletest/) (optional: for building unit tests)

The additional Python dependencies are listed in `pyproject.toml`.

#### Building the Code

1. Install C++ dependencies:

```bash
sudo apt-get update
sudo apt-get install libboost-all-dev libeigen3-dev libyaml-cpp-dev libspdlog-dev libgtest-dev libgmock-dev
```

2. Build the C++ extension and install the Python package:

```bash
pip install -v .
```

This will build the Python bindings (cr_knowledge_extraction_core) required for C++-boosted computations.

> **Note**: The `-v` flag (verbose) prints information about the build progress

**Optional:**

- To build the code in Debug mode, add the flag `--config-settings=cmake.build-type="Debug"` to the `pip` command.

See [here](https://scikit-build-core.readthedocs.io/en/latest/configuration.html#configuring-cmake-arguments-and-defines)
for further information on configuring CMake arguments via our build system (`scikit-build-core`).

> **Note**: `scikit-build-core` uses `ninja` for building the C++ extension by default.
> Thus, the build is automatically parallelized using all available CPU cores.
> If you want to explicitly configure the number of build jobs, you can do so by passing the
> flag `--config-settings=cmake.define.CMAKE_BUILD_PARALLEL_LEVEL=$BUILD_JOBS` to the `pip` command, where `$BUILD_JOBS`
> is the number of parallel jobs to use.
> See [here](https://scikit-build-core.readthedocs.io/en/latest/faqs.html#multithreaded-builds) for further details.

> **Note**: Building the package in Debug mode (see above) significantly increases the computation time of the C++
> backend. Please make sure you are building in Release mode (default setting) if you require fast computations.

### Running the Code

Run the example script `main.py`.

### Development

Check out the [README_FOR_DEVS](./readme/README_FOR_DEVS.md) for information on setting up your development environment.
