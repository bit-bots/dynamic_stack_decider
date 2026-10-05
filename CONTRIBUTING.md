# Developing DSD

Use [Pixi](https://pixi.sh) to run checks in the same locked environment as CI.
The default environment contains Python test tools and the ROS dependencies used
by the core. The visualization environment adds RQT and Qt; the format environment
contains only code quality tools.

```sh
pixi run --locked test
pixi run --locked package
pixi run --locked typecheck
pixi run --locked -e visualization typecheck-gui
pixi run --locked -e format format
pixi run --locked -e format lint
```

`format` applies fixes. Review its changes before committing. `lint` checks
formatting without applying formatter changes, following the `bitbots_main`
pre-commit stages. C++ and CMake hooks are omitted because this repository has
neither language.

When changing dependencies, edit `pixi.toml`, run `pixi install` to regenerate the
lockfile, and commit both files. Keep ROS and GUI dependencies out of the format
environment. ROS package dependencies also belong in the corresponding
`package.xml`; ordinary Python runtime dependencies belong in `setup.py`.

Both packages retain their ament Python layout and support standard Python
distribution builds. `package` builds source distributions and wheels and checks
that ROS resources are present and test fixtures are excluded.

Add focused regression tests for parser or runtime changes. Runtime tests use
small fake elements and assert observable stack and event behavior. Tests that
exercise DDS or Qt belong alongside the relevant package's tests and must run
with their real dependencies. Report hardware validation separately from these
checks.

Changes to scheduling, action outcomes, error recovery, parameter syntax, or
registration should describe compatibility and migration. Use issues to compare
options before introducing a new public contract.

The core, visualization, and packaging checks use strict mypy. Public packages
ship `py.typed`; blackboards are generic and parameter values use `object` to
reflect the existing YAML and ROS inputs. Narrow a parameter with `isinstance`
before using it as a particular type. Untyped ROS integrations are confined to
third-party imports, and the small RQT stubs document the interfaces used here.
