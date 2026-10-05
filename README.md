# Dynamic Stack Decider

DSD selects persistent actions through a hierarchy of decisions that can be
reevaluated as the environment changes. The active stack records the decisions
behind the current action, making it useful for inspecting robot behavior.

Python classes implement decisions and actions. A `.dsd` file defines their
control flow:

```text
-->HeadBehavior
$BallSeen
    YES --> @TrackBall
    NO --> @SearchBall
```

`BallSeen.perform()` returns an outcome such as `YES` or `NO`.
`TrackBall.perform()` and `SearchBall.perform()` perform one tick of work and
return nothing. Actions retain their instance state between updates and complete
with `return self.pop()`.

## Try the example

Install [Pixi](https://pixi.sh), then run the checked-in example:

```sh
pixi run --locked demo
```

The example uses a shared blackboard and switches from searching to tracking.
It requires neither a ROS node nor hardware. The core currently imports ROS
Python libraries; Pixi supplies those dependencies.

## Define elements

The blackboard can be any application object. Use the generic base classes to
keep its interface visible to the type checker:

```python
from dataclasses import dataclass

from dynamic_stack_decider import AbstractActionElement, AbstractDecisionElement


@dataclass
class Blackboard:
    ball_seen: bool = False


class BallSeen(AbstractDecisionElement[Blackboard]):
    def perform(self, reevaluate: bool = False) -> str:
        return "YES" if self.blackboard.ball_seen else "NO"

    def get_reevaluate(self) -> bool:
        return True


class SearchBall(AbstractActionElement[Blackboard]):
    def perform(self, reevaluate: bool = False) -> None:
        # Start or monitor the application's search operation here.
        pass

    def on_pop(self) -> None:
        # Cancel outstanding work and release resources here.
        pass
```

The constructor receives the shared `blackboard`, the owning `dsd`, and a
`parameters` dictionary. A subclass overriding it must call `super().__init__`.
Parameter values are YAML- or ROS-provided objects; narrow them with `isinstance`
before treating them as a specific type. Each activation receives its own copy.

## Run a behavior

Create a `DSD(blackboard)` instance, register actions and decisions, then load a
behavior. Registries map class names to their implementations; alternatively,
`register_actions(path)` and `register_decisions(path)` discover classes in a
Python file or package. The [runnable example](examples/demo.py) shows explicit
registries.

Call `update()` regularly from one owner. It reevaluates eligible decisions from
the bottom of the active stack, then follows decision outcomes immediately until
an action runs. Actions in a comma-separated chain advance on completion; the
successor runs on the next update.

Use `on_pop()` for cleanup on completion, preemption, interruption, and behavior
replacement. Return immediately after calling `pop()` or `interrupt()`.
`dsd.last_error` exposes the first failure of the latest update; ordinary update
failures are logged. Applications must choose their recovery behavior.

See the [execution contract](docs/runtime.md) for timing, cleanup, suppression,
root behavior, and error semantics. The [language reference](docs/language.md)
covers action chains, `ELSE`, comments, parameters, and reusable subtrees. Parser
errors include the source file and line.

## Inspect a running DSD

Pass both a ROS node and a debug topic namespace:

```python
dsd = DSD(blackboard, node=node, debug_topic="behavior/debug")
```

The RQT visualization discovers DSD instances through their debug topics and
shows the behavior tree, active path, and debug data. Start the installed
visualization package with:

```sh
ros2 run dynamic_stack_decider_visualization dsd_gui
```

Debug topics use JSON in `std_msgs/msg/String`. The tree and current-action topics
use transient-local durability. Element IDs identify nodes within a loaded tree;
they are not persistent IDs across process restarts.

Elements can attach debug values with `publish_debug_data(label, value)` and clear
them with `clear_debug_data()`. Unsupported values are represented as strings.
Debug failures cannot prevent behavior execution.

## Install and develop

Both packages retain their ROS 2 ament Python layout and support source and wheel
builds. In a ROS workspace, use the workspace's `rosdep` and `colcon` workflow.
ROS dependencies are declared in `package.xml`; Python dependencies are declared
in `setup.py`. Installing a wheel alone does not install ROS.

This repository's locked Pixi environments provide core testing, optional Qt/RQT
checks, and lightweight formatting tools. See [CONTRIBUTING.md](CONTRIBUTING.md)
for commands and dependency updates. The default test environment avoids the GUI
dependencies; the visualization CI job runs offscreen Qt and Graphviz tests.

## Background and open decisions

The architecture is described in
[DSD — Dynamic Stack Decider](https://link.springer.com/article/10.1007/s12369-021-00768-8).
The [Bit-Bots workspace](https://github.com/bit-bots/bitbots_main) contains body
behavior, humanoid control, and localization examples.

Architectural choices are tracked separately:

- [Failure recovery and action outcomes](https://github.com/bit-bots/dynamic_stack_decider/issues/115)
- [ROS integration and registration boundaries](https://github.com/bit-bots/dynamic_stack_decider/issues/116)
- [Parameter grammar and outcome validation](https://github.com/bit-bots/dynamic_stack_decider/issues/117)
- [Root-chain completion and suppression scope](https://github.com/bit-bots/dynamic_stack_decider/issues/121)

The maintenance changes preserve existing chain timing and suppression behavior
while tightening invalid-input handling and lifecycle guarantees.
