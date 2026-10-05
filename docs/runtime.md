# Execution contract

A loaded behavior initially contains only its root instance. `update()` first
checks reevaluating decisions from the bottom of the active stack upward, then
executes the current frame. Decision outcomes descend immediately through further
decisions to an action. An action runs at most once per update.

Actions retain their instance state between updates. `return self.pop()` completes
an action. If the action has a chain successor, its cleanup runs before the
successor is initialized, and that successor runs on the next update. Without a
successor, the previous decision becomes current and decides on the next update.
The only remaining frame cannot pop itself.

A reevaluated decision whose selected branch changes cancels every frame above
it, then initializes and executes the newly selected path in the same update.
Different outcomes handled by the same `ELSE` branch do not rebuild the path.
An invalid outcome is reported before the existing path is canceled.

`interrupt()` cancels all active frames, including the old root, and initializes
a fresh instance of the declared start element. It runs on the next update.
A callback that pops, interrupts, or otherwise changes the stack cancels the
remainder of the current traversal; its returned decision outcome is discarded.
Return immediately after `pop()` or `interrupt()` to avoid continuing work on an
instance that has left the active stack.

`on_pop()` runs once for each discarded instance, in order from the top down.
Frames are detached before cleanup. Cleanup exceptions are logged, and cleanup
continues for the remaining detached frames. If cleanup itself changes the stack,
the pending transition does not overwrite that change. Constructors initialize
instance state; they must not change the DSD stack. Each activation receives an
independent copy of its parameters.

Behavior loading parses and binds the replacement before canceling the old
behavior. Parse and registration errors therefore leave the active behavior
intact. Once cleanup starts, replacement construction can still fail; callers
must handle load errors. Do not rely on rollback of already canceled hardware
operations.

Ordinary update failures retain the existing log-and-continue policy. The first
error of an update is available as `dsd.last_error`; a successful subsequent
update clears it. An empty or non-string decision outcome is a normal library
error and does not terminate the host process. Update errors do not imply that
an action succeeded, was retried safely, or was rolled back. Error recovery and
explicit action outcomes remain under discussion in
[issue #115](https://github.com/bit-bots/dynamic_stack_decider/issues/115).

`do_not_reevaluate()` suppresses the next reevaluation. Actions with `r:false` or
`reevaluate:false` renew suppression while running. Completing a normal frame
clears suppression; advancing a chain retains it. Interruption and behavior
replacement clear it. Root-chain completion and suppression scope remain under
discussion in [issue #121](https://github.com/bit-bots/dynamic_stack_decider/issues/121).

Debug snapshots describe the state after the update. Debug failures are logged
and recorded in `last_error` without preventing behavior execution. The tree and
current-action topics use transient-local durability, so matching late
subscribers receive their latest values. An empty current-action name means that
no action is active. Debugging requires both a topic and a ROS node; otherwise it
is disabled.

DSD is synchronous. Call `update()` from one owner, avoid blocking work in
`perform()`, and marshal external changes through that owner. Recursive updates
are rejected. The stack returned by `get_stack()` is for inspection and must not
be mutated by callers.
