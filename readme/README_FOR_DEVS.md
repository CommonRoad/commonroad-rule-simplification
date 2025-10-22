## Using Pre-Commit Hooks

This project uses [pre-commit](https://pre-commit.com/) to ensure that formatters and linters automatically run when
committing files.
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

### With `uv`

1. Install the C++ dependencies as described in the [README](../README.md#third-party-dependencies).

2. Install the package without build isolation

```bash
uv sync -v --no-build-isolation-package cr_rule_simplification
```

Sometimes, it can be necessary to force a reinstallation of the package:

```bash
uv sync -v --no-build-isolation-package cr_rule_simplification --force-reinstall-package cr_rule_simplification
```


### Without `uv`

1. Install the C++ dependencies as described in the [README](../README.md#third-party-dependencies).

2. Install the Python build dependencies (required to make `--no-build-isolation` work in the next step):

```bash
pip install --group build
```
The `--group` option is only available with `pip>=25.1`.
Otherwise, install the dependencies from the `build` group manually as listed in [`pyproject.toml`](../pyproject.toml).

Also consider installing `ninja` to speed up the build process by parallelizing the compilation of C++ files:

```bash
pip install ninja
```

3. Build the package and install it in editable mode with automatic rebuilds.

```bash
pip install -v --no-build-isolation -e .
```

Note that this is considered experimental by `scikit-build-core` and is subject to change.
For more information, please see
the [documentation](https://scikit-build-core.readthedocs.io/en/latest/configuration.html#editable-installs)
of `scikit-build-core`.
Flags:

- `-v` (verbose) prints information about the build progress
- `--no-build-isolation` disables build isolation, which means the build runs in your local environment
- `-e` (editable) installs the package in editable mode

## Debugging the C++ Code

1. Install the package in editable mode using a Debug build:

```bash
uv sync -v --no-build-isolation-package cr_rule_simplification --config-settings-package cr_rule_simplification:cmake.build-type="Debug"
```

Or, without `uv`:

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
Make sure to use the Python executable from your Anaconda environment.
You do not need to set up an external build tool in CLion, i.e., you can skip step 5.
Moreover, remove "Build" from the "Before launch" section in the run configuration.
CLion will now just use your manual debug installation with automatic rebuilds enabled from above.

> **Note:** Do not disable the automatic rebuilds.
> Otherwise, CLion appears to not recognize the breakpoints you set.
> It also appears that breakpoints are not recognized if you start debugging immediately after changing the code.
> In this case, restarting the debugging session should help.
> If all else fails, uninstalling and reinstalling the package also seems to fix the breakpoint recognition.

## Building the Python Bindings Manually

For development purposes it can be convenient to set up your IDE to build the Python bindings without invoking `pip`.
To this end, you need to manually enable the Python bindings by passing `-DCR_REACH_FLOW_BUILD_PYTHON_BINDINGS=ON`
when configuring the CMake project.
If you do so, it is also necessary to install the Python build dependencies as described above.
Finally, you need to point CMake to the correct Python `site-packages` directory by specifying
`-DCMAKE_PREFIX_PATH=/path/to/site-packages` when configuring the CMake project.
If you are using a Conda environment, this path is typically `${CONDA_PREFIX}/lib/python3.10/site-packages`.
Note that the Python version may vary depending on your setup.
