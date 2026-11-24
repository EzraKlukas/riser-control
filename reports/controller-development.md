---
id: controller-development
aliases:
  - chainable-controller-reference
tags: []
---
# Writing a ros2_control controller

A checklist for an `ament_cmake` C++ controller package, with experiment logic
kept separate from ROS integration. Run the commands from the repository root.
Replace the example names with your package, class and controller instance.

## 1. Define the contract and choose the base class

Specify inputs and outputs by full interface name, units, coordinate frame,
timestamp clock and validity policy. Decide which commands the controller owns,
what happens on missing/stale/nonfinite input, and which settings can change
while active. Establish the update rate and data dependencies before coding.

| Choice | `ControllerInterface` | `ChainableControllerInterface` |
| --- | --- | --- |
| Consume hardware or another controller's states | Yes | Yes |
| Command hardware or another controller's exported references | Yes | Yes |
| Export your own states or writable references | No | Yes |
| Main calculation callback | Override `update(time, period)` | Implement `update_and_write_commands(time, period)`; base `update()` is final |
| External reference input | Your own subscriber/handoff if needed | `update_reference_from_subscribers()` runs when not in chained mode |

**Chainable** describes the ability to export interfaces. **Chained mode** is a
runtime mode entered when another controller claims this controller's exported
reference interfaces. There is no separate “normal chained controller” base
class. Both base classes share `ControllerInterfaceBase`.

Keep these two connections distinct:

- **Exported state:** the producer writes a measurement/result; consumers read
  it through their state interfaces. Multiple consumers are allowed. Reading
  these states does not switch the producer into chained mode.
- **Exported reference:** another controller writes a reference through its
  command interfaces. The receiving controller is chainable and switches command
  input ownership to the chain. Its external command-input path must not compete
  with that writer; unrelated publishers can continue.

A controller that only consumes exported states can use `ControllerInterface`.
Choose the chainable base when it must export interfaces of its own. See the
[interface API](https://control.ros.org/jazzy/doc/api/classcontroller__interface_1_1ChainableControllerInterface.html)
and [chaining semantics](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/controller_chaining.html).
Use the installed headers for exact signatures.

## 2. Create the package and registration files

```text
src/experiment_controller/
  package.xml
  CMakeLists.txt
  plugins.xml
  include/experiment_controller/experiment_controller.hpp
  include/experiment_controller/experiment_logic.hpp
  src/experiment_controller.cpp
```

Add a math header/source only when it makes the current logic easier to read.
Put robot-specific runtime settings in the existing bringup configuration.

| File | Purpose |
| --- | --- |
| `package.xml` | Package identity and dependency metadata for colcon, rosdep and installation. It describes what the package needs; it does not set compiler include paths. |
| `CMakeLists.txt` | Builds the shared library, supplies compiler/linker settings, installs files and registers plugin metadata with ament. |
| `plugins.xml` | Lets pluginlib map a runtime lookup name to a C++ class, base type and shared library. It does not compile the class or create its factory. |
| Controller `.hpp/.cpp` | ROS lifecycle, parameters, loaned interfaces, exported storage and the plugin factory registration. |
| Logic `.hpp` | Plain input/output/configuration types, validation and experiment calculations, without ROS handle types. |

Keep these names consistent; they serve different purposes:

| Example | Meaning |
| --- | --- |
| `experiment_controller` | Package name; matches `project()` and `<name>`. |
| `experiment_controller_plugin` | CMake library target and `plugins.xml` library path. |
| `experiment_controller::ExperimentController` | Fully qualified C++ class. |
| `experiment_controller/ExperimentController` | Plugin lookup name used by the YAML `type` field. |
| `experiment` | Runtime instance name; prefixes its node and exported interfaces. |

A minimal manifest is:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>experiment_controller</name>
  <version>0.1.0</version>
  <description>Experiment controller.</description>
  <maintainer email="maintainer@example.com">Your name</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>controller_interface</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

Replace the maintainer and license with the actual project values. Declare direct
message/math dependencies when used. `<depend>` covers build, export and runtime
use; use narrower dependency tags when appropriate. Bringup separately needs an
`<exec_depend>` on this package because it loads the plugin at runtime.

A matching CMake file is:

```cmake
cmake_minimum_required(VERSION 3.16)
project(experiment_controller)
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(experiment_controller_plugin SHARED src/experiment_controller.cpp)
target_include_directories(experiment_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(experiment_controller_plugin PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(experiment_controller_plugin PRIVATE -Wall -Wextra -Wpedantic)
endif()
ament_target_dependencies(experiment_controller_plugin
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(controller_interface plugins.xml)
install(TARGETS experiment_controller_plugin LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_package()
```

`add_library(... SHARED ...)` creates the loadable `.so`; list additional `.cpp`
files there when used. Headers included by those sources need no separate compile
target. `find_package` locates dependencies; `ament_target_dependencies` applies
their include/link requirements to this target. Install rules make the library
and headers available in the sourced workspace. The pluginlib export macro
installs/indexes `plugins.xml`; `ament_package()` finishes package registration.
If other packages must link to this library at compile time, also export its CMake
usage requirements/targets. Runtime plugin loading alone does not require that.

For a chainable class, use:

```xml
<library path="experiment_controller_plugin">
  <class name="experiment_controller/ExperimentController"
         type="experiment_controller::ExperimentController"
         base_class_type="controller_interface::ChainableControllerInterface">
    <description>Experiment controller.</description>
  </class>
</library>
```

At the end of the compiled controller `.cpp`, outside its namespace:

```cpp
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    experiment_controller::ExperimentController,
    controller_interface::ChainableControllerInterface)
```

For an ordinary controller, use `controller_interface::ControllerInterface` in
both places and inherit from that class. The C++ macro supplies the factory;
`plugins.xml` describes it; CMake makes it discoverable. A YAML `type` entry alone
cannot register a plugin.

## 3. Keep the ROS adapter thin

The update path should read **plain inputs → logic update → plain outputs**.
The controller owns loaned handles and ROS resources. The logic owns named value
types such as `Inputs`, `Output` and `Configuration`, plus validation and control
flow. Keep reusable mathematics separate when substantial or shared.

| Callback | Responsibility |
| --- | --- |
| `on_init()` | Declare parameters; initialize safe output storage and retained callbacks. |
| `on_configure()` | Parse/validate parameters and models; prepare fixed buffers and cached settings. |
| `state_interface_configuration()` | Return `INDIVIDUAL` with every required state name, including metadata. |
| `command_interface_configuration()` | Declare command/reference resources to write; use `NONE` when none are needed. |
| `on_activate()` | Find/check the loaned handles by name, bind member references and reset experiment state. |
| `update()` or chainable update hook | Read a snapshot, delegate to logic, then write commands/exported output. |
| `on_deactivate()` / `on_cleanup()` | Apply the defined stop policy, invalidate output and clear borrowed references. |

The manager supplies loaned interfaces before `on_activate()`; that callback is
where you normally locate them and connect your internal fields. Do not infer
ordering from the loaned vector. Check `get_optional()` results when reading and
honor failed command-write results through an explicit fault path.

Read and cache parameters during configuration. Retain parameter callback handles
and reject unsafe active edits. A successful `ros2 param set` does not refresh
cached logic settings: deactivate and reconfigure, or unload/respawn. Interface-name
or count changes may require reload with dependent controllers stopped.

Decide whether activation requires good feedback or may start with invalid output
until measurements arrive. A state-only provider can often recover from missing
data without deactivation. Motion-producing controllers need an explicit safe
response to lost feedback; returning `OK` with invalid output is not a universal
policy. A reported controller error does not itself guarantee physical stopping.

## 4. Add chainable-specific hooks when needed

Include `controller_interface/chainable_controller_interface.hpp` and implement:

| Hook | Responsibility |
| --- | --- |
| `on_export_state_interfaces_list()` | Return shared state handles backed by stable output storage, or an empty list. |
| `on_export_reference_interfaces_list()` | Return shared command handles backed by reference-input storage, or an empty list. |
| `update_reference_from_subscribers(time, period)` | Copy the latest external reference into control-loop storage when not chained. Return `OK` for a state-only provider with no such input. |
| `update_and_write_commands(time, period)` | Run calculations and publish commands or exported states. The name also applies to state-only providers. |
| `on_set_chained_mode(bool)` | If needed, switch reference-input ownership and reject unsupported transitions. |

The current state-export hook returns
`std::vector<hardware_interface::StateInterface::SharedPtr>`; the reference hook
returns `std::vector<hardware_interface::CommandInterface::SharedPtr>`. Use the
installed header rather than copying older export signatures. Base `update()`
handles the chainable sequence; do not override it.

Prefix exported handles with `get_name()`, e.g. `experiment/value`, so multiple
instances work. Keep their backing storage alive and at stable addresses; never
export a pointer to a local or resize a backing vector later. Exported states are
written by the producer. In chained mode, exported references are written by an
upstream controller; otherwise the external-input handoff can populate them.
Neither requires a hardware declaration in the URDF.

Subscriber callbacks should hand off reference values safely; they should not
write interface storage concurrently with the control loop. Reading exported
states does not disable the producer's subscribers or make it “chained.”

## 5. Consume another controller's interfaces

**Activation order alone is not enough.** A state consumer must:

1. Declare exact input names in `state_interface_configuration()`, for example
   `measurement/value`, `measurement/sample_time` and `measurement/valid`.
2. Locate the supplied handles during activation and retain borrowed references.
3. Read/check their values during updates, including quality and timestamp rules.

Those are the usual steps for hardware states too. No special chainable base,
subscriber or export hook is needed merely to read another controller's states.

Load, configure and activate the producer first, then configure and activate
its consumers. This also lets broadcasters discover available exported states. Stop all dependent consumers before
stopping, reconfiguring or unloading the producer. The manager derives
relationships from interface declarations; activation order alone does not
establish update order. Check producer-before-consumer execution, especially with
different rates or asynchronous controllers. Freshness must still be checked.

A consumer selecting `ALL` states also counts as a dependency; the general joint
state broadcaster can therefore prevent a producer from being stopped alone.
An already active broadcaster does not automatically acquire newly exported handles.

For reference chaining, the writer instead declares `receiver/reference` in its
**command** interface configuration. Activate the reference-exporting receiver
before its writer; the update dependency runs writer → receiver. For state flow,
the calculation dependency runs state producer → state consumer. Avoid cycles.

## 6. Wire bringup and inspect the chain

Register the type and instance parameters in the controller YAML:

```yaml
controller_manager:
  ros__parameters:
    experiment:
      type: experiment_controller/ExperimentController
experiment:
  ros__parameters:
    # Add only parameters that this controller declares and reads.
    gain: 1.0
```

Add the package as a bringup runtime dependency and add its instance to the
spawner sequence after its providers. Pass the parameter file explicitly:

```bash
ros2 run controller_manager spawner measurement experiment \
  --param-file src/riser_bringup/config/controllers.yaml
ros2 control list_controllers --verbose
ros2 control list_hardware_interfaces
ros2 control view_controller_chains
ros2 param dump /experiment
ros2 param get /experiment gain
ros2 param set /experiment gain 0.5
```

The YAML must also contain the producer's type entry and parameters. The spawner
loads, configures and activates the named instances in sequence. Active parameter
edits may be rejected by design. Use `list_controller_types` when a
plugin cannot be loaded. Keep the manager's robot-description input and namespace
consistent with the launch configuration.

State-interface memory is not itself a ROS topic. Use an appropriate broadcaster
or explicit diagnostic publisher for plotting. PlotJuggler consumes the published
topics; an active broadcaster's interface selection determines what it exposes.

## 7. Build and make the editor understand the package

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to experiment_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

`--packages-up-to` builds local prerequisites too. Source the resulting install
before trying plugin discovery or launch. Include each real `.cpp` in a CMake
target and declare its dependencies; editor settings cannot repair a broken build.

*Editor-specific: the following is for clangd, the C++ language server. It is not
a controller requirement. It applies to any editor using clangd, including VS Code
with its clangd extension; other language servers have their own configuration.*

Clangd needs the compilation database to learn include paths, compiler defines
and the C++ standard. `CMAKE_EXPORT_COMPILE_COMMANDS=ON` generates
`build/experiment_controller/compile_commands.json`. Using clangd does not require
building the package with Clang; GCC compile commands can also be used.
[Compilation database explanation](https://clangd.llvm.org/design/compile-commands).

The root [.clangd](../.clangd) uses conditional per-package rules. For a new
package, add a block like this to that file (the paths are relative to its location):

```yaml
---
If:
  PathMatch: src/experiment_controller/.*
CompileFlags:
  CompilationDatabase: build/experiment_controller
```

This points clangd at the directory containing the database, not the JSON file.
Rebuild after changing source lists/dependencies and restart the language server
if it retains old diagnostics. Headers normally borrow a command from a source
file that includes them; include the logic header from the built controller.
[Clangd configuration](https://clangd.llvm.org/config).

Check the setup independently of the editor:

```bash
clangd --check=src/experiment_controller/src/experiment_controller.cpp \
  --compile-commands-dir="$PWD/build/experiment_controller" --check-locations=false
```

`--check-locations=false` skips clangd's extra per-token editor-feature self-tests
while retaining compiler-command and parser checks. Without it, an
`ExpandDeducedType` tweak failure can be reported separately from C++ diagnostics.
See [clangd's check-mode implementation](https://github.com/llvm/llvm-project/blob/release/18.x/clang-tools-extra/clangd/tool/Check.cpp#L477).

If functions remain unresolved, distinguish missing `#include`, missing CMake
source/dependency, wrong API signature/base class, stale database and compiler/
standard-library mismatch. Check the loaded compile command in clangd's log.
Sourcing ROS alone does not provide a compilation database or reconfigure an
already running language server. Hard-coded editor include-path lists tend to
hide the actual build settings.

For `bottom_csv_system.cpp`, use the IgH-enabled `riser_hardware` build
(`RISER_WITH_IGH=ON`). If that build already exists, source the ROS environment
and regenerate its compilation database from the workspace root:

```bash
cmake -S src/riser_hardware -B build/riser_hardware -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

The compile command for `bottom_csv_system.cpp` then includes the IgH header
directory detected by CMake.

## 8. Timing and handoff checks

Keep allocation, model/YAML parsing, parameter calls, blocking I/O, blocking
mutexes and routine logging out of the update path. Prepare storage during
configuration and use bounded handoffs between callback and control threads.
Confirm scheduling and thread ownership rather than assuming hard realtime.

Use an explicit clock contract. A timestamp's epoch matching another timestamp
does not guarantee both represent the same current cycle. The simulated hardware
exports `RiserSystem/read_time`; use it when applicable for age checks and elapsed
time. Physics steps, controller cycles, sensor acquisition and message arrival
are distinct events. Specify behavior on pause, reset, stale data and rate mismatch.

Before handoff, build with warnings enabled; run existing relevant checks and a
live producer/consumer check. Verify discovery, activation dependencies, handle
names, update order, normal commands, invalid input and repeated lifecycle use.
Use focused tests/probes appropriate to interface stability; keep temporary
captures out of the repository. Update operating documentation when behavior or
configuration changes.

For concrete local examples, see [the regular calibration controller](../src/riser_calibration),
[the chainable wrench controller](../src/riser_wrench_aggregation), and
[bringup configuration](../src/riser_bringup/config/README.md).
