# Behavior language

A file contains named subtree definitions and one behavior entry point. Subtrees
must be defined before they are referenced. `-->` introduces the main behavior;
an optional name can follow it, such as `-->HeadBehavior`. Each definition has one
unindented root expression.

```text
#Search
@LookAround, @SearchBall

-->HeadBehavior
$BallSeen
    YES --> @TrackBall
    NO --> #Search
```

Decisions start with `$`, actions with `@`, and subtree calls with `#`. Each
decision outcome has one branch. Use `ELSE` to handle otherwise unmapped outcomes.
Names resolve to registered Python classes when the behavior is loaded.

Indent each decision's branches one level deeper than its expression. Use spaces
in groups of four. An action or subtree call cannot receive further branches.
Only a decision written directly at the end of an action chain can receive
branches on following lines:

```text
-->
$Mode
    ACTIVE --> @Prepare, $BallSeen
        YES --> @TrackBall
        NO --> @SearchBall
    ELSE --> @Idle
```

Comma-separated expressions form a chain. Every expression except the last must
be an action; the last can be an action, decision, or subtree call. Completing an
action advances to its successor. Execution timing is defined by the runtime.

Parameters use `+ key:value`. Literal values are decoded with `yaml.safe_load`.
`%name` looks up a ROS parameter and requires a node. Subtree definitions declare
parameter names with `+ name`, and elements within them reference those values
with `*name`:

```text
#TimedSearch + duration
@SearchBall + time:*duration

-->
$BallSeen
    YES --> @TrackBall
    NO --> #TimedSearch + duration:10
```

The existing parameter syntax does not provide general escaping for DSL
separators. Spaces in parameter values are rejected; commas, colons, plus signs,
and comment markers have syntactic meaning. The parameter grammar and optional
schemas are being discussed in [issue #117](https://github.com/bit-bots/dynamic_stack_decider/issues/117).

`//` starts a line comment. Block comments start with `//**` and end with `**//`;
they can share lines with code. A block comment must be closed.

The parser rejects missing or duplicate entry points, duplicate roots, branches,
subtrees or parameters, invalid indentation, decisions without branches,
incomplete definitions, unresolved references, and malformed YAML values.
`ParseError` includes the file and line. Parsed elements retain their definition
locations in `source_file` and `source_line`. Invalid behavior files should be
fixed before loading them into a running agent.

The legacy subtree alias form `RESULT --> #OtherSubtree` is accepted as the
unindented body of a subtree definition. It copies the referenced subtree and
resolves its parameters. New behavior files can normally call the referenced
subtree directly.
