## CommonRoad Rule Simplification: Extract Scenario-Specific Knowledge to Simplify Traffic Rules

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

**Optional dependencies:**

- [GTest](https://google.github.io/googletest/) (optional: for building unit tests)

The additional Python dependencies are listed in `pyproject.toml`.

#### Building the Code

> **Note:** If you want to use this package with the PyPI versions of its dependencies, you need to compile it using GCC 10 (which is the compiler we use to create the PyPI wheels).
> Otherwise, nanobind will not be able to detect the Python bindings of the dependencies correctly (see [here](https://nanobind.readthedocs.io/en/latest/faq.html#how-can-i-avoid-conflicts-with-other-projects-using-nanobind)).
> To do so, indicate the path to GCC 10 in the `CXX` environment variable before building the code (e.g. `export CXX=/usr/bin/g++-10`).
> Note that you need to start with a fresh build directory if you change the compiler, as CMake caches the compiler used for the build.
> Alternatively, if you cannot use GCC 10 for some reason, you can install the following packages from source using the compiler of your choice:
> [commonroad-clcs](https://github.com/CommonRoad/commonroad-clcs).
> Make sure to use the correct versions of these packages as specified in the `pyproject.toml` file.

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

### Possible Installation Problems

- Protobuf error like this `CHECK failed: GeneratedDatabase()->Add(encoded_file_descriptor, size)`:
  this is caused by the `commonroad_cpp` Python package and the version of the environment model used by our C++ code
  trying to register the same type with the same protobuf instance twice. To work around this issue, you can add the
  `--config-settings=cmake.define.COMMONROAD_SYSTEM_PROTOBUF=OFF` option to the `pip install` command.

### Documentation

To generate the documentation, first make sure that you have installed the documentation dependencies listed in `pyproject.toml`.
Then, you can generate the documentation by running:
```bash
mkdocs build
```

To view the documentation, you can start a local server by running:
```bash
mkdocs serve
```

### Development

Check out the [README_FOR_DEVS](./readme/README_FOR_DEVS.md) for information on setting up your development environment.
