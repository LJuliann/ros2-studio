# ROS 2 Studio IPC protocol v1

`ros-studio-daemon` reads one UTF-8 JSON object per line from standard input and writes one JSON object per line to standard output. Standard output is reserved for protocol messages; diagnostics belong on standard error. Each response is flushed immediately. Zed is not connected to this daemon yet; M09 establishes the local transport contract.

Every message has `"jsonrpc":"2.0"` and `"protocol_version":1`. The JSON-RPC version identifies the envelope format; `protocol_version` identifies the ROS 2 Studio message schema. Incompatible protocol versions are rejected rather than silently interpreted.

Requests carry an unsigned integer `id`, a snake-case `method`, and an object `params`. Responses repeat the `id` and contain exactly one of `result` or `error`. Events are JSON-RPC notifications: they have `method` and `params`, but no `id`.

## Requests

| Method | Parameters |
| --- | --- |
| `open_workspace` | `path: string` |
| `start_runtime_discovery` | none (`{}`) |
| `stop_runtime_discovery` | none (`{}`) |
| `launch` | `command: string[]`, `env: {string: string}` |
| `get_parameters` | `node: string` |
| `set_parameter` | `node: string`, `name: string`, `value: JSON value` |

`launch.command` is an argument vector, not a shell command. The daemon must not interpret it through a shell.

Successful results currently use `{"kind":"accepted"}`. The protocol also defines `parameters` with a `values` object and `process_started` with an `id` string for later milestones.

## Events

| Method | Parameters |
| --- | --- |
| `static_graph_changed` | `patch: GraphPatch` |
| `runtime_graph_changed` | `patch: GraphPatch` |
| `process_output` | `id`, `stream` (`stdout` or `stderr`), `text` |
| `diagnostic` | `severity` (`info`, `warning`, or `error`), `message`, optional `source_location` |

`GraphPatch` identifies its `project_id` and carries `upsert_packages`, `removed_package_ids`, `upsert_nodes`, and `removed_node_ids`. Each upsert contains a complete model entity. Removal IDs refer to previously sent entities. Consumers apply a patch atomically and ignore removals of already absent entities.

## Dummy daemon

M09 provides the transport and schema, not ROS discovery. The dummy daemon acknowledges `open_workspace` and `stop_runtime_discovery`; runtime discovery, launch, and parameters return capability-unavailable error `-32002`. A later milestone will replace these stubs without changing the v1 envelope.

To compile and smoke-test the daemon:

```sh
printf '%s\n' '{"jsonrpc":"2.0","protocol_version":1,"id":1,"method":"open_workspace","params":{"path":"/tmp/drone_ws"}}' | CARGO_BUILD_JOBS=1 cargo run -p ros_studio_daemon --bin ros-studio-daemon
```

Expected response:

```json
{"jsonrpc":"2.0","protocol_version":1,"id":1,"result":{"kind":"accepted"}}
```

The daemon does not check that the example path exists, scan the workspace, or start ROS in M09.
