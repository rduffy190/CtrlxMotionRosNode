# motion_action_cpp

ROS 2 bridge to the ctrlX MOTION app. Discovers the configured axes at startup
and, for each one, exposes:

| Interface | Type | Name |
|---|---|---|
| Topic (publish) | `axis_status_msg/AxisStatus` | `<node>/<axis>/status` |
| Action | `ctrlx_motion_action/PowerOn` | `<node>/<axis>/power_on` |
| Action | `ctrlx_motion_action/ResetFault` | `<node>/<axis>/reset_fault` |
| Action | `ctrlx_motion_action/MovePosition` | `<node>/<axis>/move_position` |

## Design

**Two Data Layer clients, one session.** A single `DataLayerSession` owns the
process-wide `DatalayerSystem`; two `SharedDataLayerClient` instances are
created from its factory, each with its own `IClient3` connection:

| Client | Used by | Contention |
|---|---|---|
| `status` | the poll timer's bulk read (plus discovery/diagnostics at startup) | effectively none — one caller |
| `action` | every action server on every axis | commands from different axes queue here |

A slow status read therefore cannot delay a command, and a burst of commands
cannot stall status publication. Two connections total, regardless of axis
count.

Each client guards its `IClient3` with a plain blocking `std::mutex`. The
contract the action servers rely on is exactly: **lock → request → unlock.** A
caller that cannot take the lock waits until it can — no try-lock, no timeout,
no failure path. `write_json` holds the lock across its internal metadata
lookup, type read and write so a command is never interleaved with another
thread's command; after the first call per address both lookups are cached, so
the steady-state cost is a single write.

The shared JSON converter and its mutex are gone. They existed only because
commands were driven from JSON payloads while the MOTION schemas were
unavailable; with typed flatbuffers there is no shared mutable state on the
command path at all.

Set `lock_stats_period_s` to a non-zero value to have the node periodically log
request counts and the worst mutex wait per client — useful for sizing
`status_rate_hz` against your command load.

Nothing here is real-time safe, by design — the motion kernel owns the real-time
behaviour and this node is the supervisory layer.

**One bulk read for all axes.** A single timer builds one `VecBulkRequest`
containing every node of every axis and issues one `readBulkSync`. Position,
velocity and acceleration all arrive in a single `AxsIpoValues` flatbuffer, so
each axis contributes only two nodes (IPO values + op-state). At 20 Hz with 8
axes that is 20 round trips per second carrying 16 nodes.

**IPO values, not actual values.** `AxsActualValues` documents `actualVel`,
`actualAcc` and `distLeft` as *"currently not supported for real drives"*. Using
it would pin `AxisStatus.act_vel` at zero on hardware and silently reduce the
move-completion check to position-only — declaring success while the axis was
still moving through the target. `AxsIpoValues` is the interpolator setpoint
stream and is always populated, so completion means "the interpolator finished
the commanded motion". It does not account for following error; read the actual
values node as well if you need that. The response is ordered identically to the request,
so each axis decodes its own slice by offset.

**Commands are typed flatbuffers.** The ctrlX MOTION schemas are vendored into
`include/motion_action_cpp/fbs/` (see the README there), so `AxsCmdPosData`,
`SysCmdReset` and `AxsCmdAbortData` are built with the generated `Create*`
functions and field names are checked by the compiler. There is no JSON, no
runtime schema lookup and no shared converter on the command path.

Two schema defaults are actively dangerous and are therefore always sent
explicitly: `AxsCmdPosData.buffered` defaults to **true** (which would queue
every move behind current motion instead of running it), and
`DynamicLimits.vel/acc/dec` and `AxsCmdAbortData.dec` default to **1.0** rather
than 0 (so an omitted limit is a crawl, not an error).

### Vendored flatbuffer schemas

`include/motion_action_cpp/fbs/motion/core/` holds five generated headers copied
verbatim from the ctrlX AUTOMATION SDK flatbuffer set. They are vendored rather
than generated at build time because `flatc` and the `.fbs` sources are not
build dependencies of this package. Namespace is `motion::core::fbtypes`.

| Header | Type | Used for |
|---|---|---|
| `axsIpoValues_generated.h` | `AxsIpoValues` | status read (pos/vel/acc in one node) |
| `axsCmdPosData_generated.h` | `AxsCmdPosData` | MovePosition |
| `dynamicLimits_generated.h` | `DynamicLimits` | nested `lim` table of the above |
| `axsCmdAbortData_generated.h` | `AxsCmdAbortData` | abort on cancel |
| `sysCmdReset_generated.h` | `SysCmdReset` | ResetFault |

If the MOTION app is upgraded and a schema changes, re-copy the affected header
from the matching SDK release. Field *additions* are backward compatible in
flatbuffers; removals and renumbering are not — and would surface as a compile
error here rather than as silent misbehaviour at runtime, which is the main
argument for typed commands over JSON.

**Actions never touch the Data Layer except to command.** An accepted goal
writes its command once on the action client, then generates feedback and
evaluates completion from the cached status the poll timer already maintains on
the status client. A running move adds zero extra Data Layer traffic.

**No command timeout.** A move can legitimately take any amount of time, so
goals are not deadlined. A running goal ends when it reaches its target, faults,
loses power, is cancelled by the caller, or the axis stops reporting status
(`status_stale_timeout_s`, the one remaining automatic abort). Monitoring is the
caller's job — watch the action feedback, which streams `dist_to_target` /
`powering_on` / `resetting` continuously. The node also logs a progress line
every 10s per running goal so a stuck command is visible without a subscriber.

The trade-off to be aware of: because completion is currently inferred from
position and velocity rather than a per-command status node, a command the
motion kernel silently rejects will leave its goal running until you cancel it.
Wiring up a real done/busy node (see item 4 below) removes that.

**Actions never block the executor.** Each accepted goal runs on its own
detached worker. Action servers share a Reentrant callback group; the timer has
its own MutuallyExclusive group; `main` spins a `MultiThreadedExecutor`. One
command at a time per axis — a goal arriving while another runs is rejected.

**Commands are written as JSON, converted at runtime.** `write_json` resolves a
node's flatbuffer schema through `metadataSync` → `Metadata::references()` with
`type == "writeType"` → read the type node, then uses
`IFlatbufferJson::parseJson` to build the flatbuffer. This means no generated
`*_generated.h` headers for any motion schema are needed, and a wrong payload
produces a converter error naming the offending field.

## Build

The ctrlX Data Layer library is found either through pkg-config (the
`ctrlx-datalayer` deb — see `scripts/install-ctrlx-datalayer.sh` in the SDK) or
from an unpacked SDK archive:

```bash
colcon build --packages-select axis_status_msg ctrlx_motion_action motion_action_cpp
# or, without the deb:
colcon build --cmake-args -DCTRLX_SDK_DIR=$HOME/ctrlx-automation-sdk
```

Note: the SDK's own C++ samples compile with `-fno-rtti`. Do not copy that —
rclcpp needs RTTI. This package links the shared `libcomm_datalayer.so`, which
keeps the two ABIs apart.

## Run

```bash
ros2 launch motion_action_cpp ctrlx_motion.launch.py
```

Connection settings live in `config/ctrlx_motion.yaml`. Inside a snap on the
CORE they are ignored and `ipc://` is used automatically. For a COREvirtual with
port forwarding, set `ip: "10.0.2.2"` and `ssl_port: 8443`.

```bash
# watch an axis
ros2 topic echo /ctrlx_motion_node/AxisX/status

# power it up
ros2 action send_goal /ctrlx_motion_node/AxisX/power_on \
  ctrlx_motion_action/action/PowerOn "{power_on: true}"

# move, with feedback
ros2 action send_goal -f /ctrlx_motion_node/AxisX/move_position \
  ctrlx_motion_action/action/MovePosition \
  "{pos: 100.0, vel: 50.0, acc: 200.0, dcc: 200.0}"
```

## Filling in the TODOs

Everything I could not verify against your firmware is a parameter in
`config/ctrlx_motion.yaml`, so fixing one is an edit and a restart — no rebuild.
The node has a helper that tells you what the right values are:

```bash
ros2 launch motion_action_cpp ctrlx_motion.launch.py dump_axis_tree:=true
```

For each axis it logs the browse tree, the current value of every status node it
is configured to read (as JSON), and the **write schema** of every command node —
i.e. exactly which fields the payloads may contain.

The open items, in rough order of how likely they are to need changing:

1. **`paths.*`** — the Data Layer addresses. The schemas define payload *shape*,
   not node paths, so these are still guesses. The dump prints browse results
   and, for each command node, the `writeType` it declares.
2. **`paths.op_state` and `decode_op_state()`** — `powered_on` and `faulted` are
   derived from one scalar node via a name → bool mapping in `src/dl_paths.cpp`.
   There is no axis operation-state enum anywhere in the MOTION flatbuffer set,
   so this is genuinely unknown. The node logs any state name it doesn't
   recognise, so they're easy to collect.
3. **The power command** — no core axis power/enable schema exists in the
   vendored set. `execute_power()` writes a bare `BOOL8` Variant, which is right
   if the node is a plain boolean and wrong if it wants a flatbuffer. The dump's
   `write type cmd_power` line answers this.
4. **MovePosition completion** (`src/axis_interface.cpp`, marked in-line) — now
   `|target − ipoPos| < tol && |ipoVel| < vel_tol`. Since there is no command
   timeout, a command the kernel silently rejects leaves its goal running until
   cancelled. If the axis exposes a per-command done/busy node, add it to
   `PathTemplates`, add it to `append_status_requests()`, and gate completion on
   it — it rides in the existing bulk read, so it costs nothing.
5. **`CmdPosAbsDir`** — hardcoded `SHORTEST_WAY`; only meaningful for modulo
   axes.
6. **Axis discovery filtering** — if browsing the axis root returns non-axis
   children, filter them in `discover_axes()` or list axes explicitly via the
   `axes` parameter.

## Unrelated issue in the uploaded interface packages

`axis_status_msg/package.xml` and `ctrlx_motion_action/package.xml` both declare
the package name as `<n>...</n>` rather than `<name>...</name>`. That is not
valid `package.xml`; `catkin_pkg` will fail to parse it. Worth fixing in both
before the next clean build. (`motion_action_cpp/package.xml` in this delivery
uses `<name>`.)
