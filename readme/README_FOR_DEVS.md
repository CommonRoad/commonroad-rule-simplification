## Using Pre-Commit Hooks

This project uses [pre-commit](https://pre-commit.com/) to ensure that formatters and linters automatically run when committing files.
To use pre-commit, install it via pip:

```bash
pip install pre-commit
```

Alternatively, pre-commit is also included in the optional `dev` dependencies of this project.
Then install the pre-commit hooks so that they automatically run before each commit:

```bash
pre-commit install
```

To run the pre-commit hooks manually, use

```bash
pre-commit run --all-files
```

## Editable Install (experimental)

1. Install the C++ dependencies as described in the [README](../README.md).

2. Install the Python build dependencies (required to make `--no-build-isolation` work in the next step):

```bash
pip install scikit-build-core~=0.10.7 nanobind~=2.2.0 pathspec>=0.12.1 pyproject-metadata>=0.7.1 typing_extensions~=4.12.2 cmake>=3.24
```

> **Note:** The versions of the dependencies might have changed from the time of writing this README. Please check the
> optional build dependencies in the [`pyproject.toml`](../pyproject.toml) file for the latest versions.

3. Build the package and install it in editable mode with automatic rebuilds.

```bash
pip install -v --no-build-isolation --config-settings=editable.rebuild=true -e .
```

Note that this is considered experimental by `scikit-build-core` and is subject to change.
For more information, please see
the [documentation](https://scikit-build-core.readthedocs.io/en/latest/configuration.html#editable-installs)
of `scikit-build-core`.
Flags:

- `-v` (verbose) prints information about the build progress
- `--no-build-isolation` disables build isolation, which means the build runs in your local environment
- `--config-settings=editable.rebuild=true` enables automatic rebuilds when the source code changes (see the caveats in
  the documentation of `scikit-build-core`)
- `-e` (editable) installs the package in editable mode

## Debugging the C++ Code

1. Install the package in editable mode using a Debug build:

```bash
pip install -v --no-build-isolation --config-settings=editable.rebuild=true --config-settings=cmake.build-type="Debug" -e .
```

2. Launch the Python interpreter under a C++ debugger, for example with GDB:

```bash
gdb -ex r --args python main.py
```

You can also use your favorite IDE to debug the C++ code.

### Debugging with CLion

To set up a debugging configuration with CLion, follow the steps described
under [option 2 here](https://www.jetbrains.com/help/clion/debugging-python-extensions.html#debug-custom-py).
Make sure to use the Python and pip executables from your Anaconda environment.

When setting up the external build tool in CLion, we recommend to choose a different build directory to avoid
interference with your manual builds.
You also have to make sure that CMake uses the correct compiler version (see the note at the top of
the [README](../README.md)).
Below, you find the pip arguments of an example configuration:

```
install
-v
--no-build-isolation
--config-settings=editable.rebuild=true
--config-settings=cmake.build-type="Debug"
--config-settings=build-dir=build/CLion
-e
.
```

> **Note:** Do not disable the automatic rebuilds. Otherwise, CLion appears to not recognize the breakpoints you set.
> It also appears that breakpoints are not recognized if you start debugging immediately after changing the code.
> In this case, restarting the debugging session should help.

Alternatively, you can omit the build step in the CLion configuration and just rely on the automatic rebuilds of your
manual debug installation.
With this, the breakpoints seem to work more reliably.
To do so, edit your run configuration and remove "Build" from the "Before launch" section.

If all else fails, uninstalling and reinstalling the package also seems to fix the breakpoint recognition.
