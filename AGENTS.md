---
id: AGENTS
aliases: []
tags: []
---
# Coding guidance

## Scope and simplicity

- Make the smallest coherent change that solves the request through the normal entry point. Preserve existing behavior outside the requested change.
- Reuse existing functions and patterns where they fit. Add a helper, abstraction, or dependency only when it makes the current implementation easier to understand or removes meaningful duplication.
- Keep formatting changes local. Before finishing, review the diff and remove dead code and temporary files introduced during the task.

## Readability and precision

- Use descriptive names, normal spacing around operators and after commas, and blank lines between distinct steps. Break up dense expressions when named intermediate values make the logic easier to follow.
- Use short comments to explain decisions a reader cannot infer from the code, especially sign conventions, coordinate frames, calibration provenance, and control-loop constraints.
- Match the precision of displayed values and hand-written estimates to their source and purpose. Preserve authoritative CAD and calibration values at their source precision. Use named constants with units where they clarify the calculation.

## Failure handling

- Handle failures that can occur in the supported workflow, such as missing configuration, invalid external input, lost feedback, and failed I/O. Validate at the boundary and use the existing error paths.
- Give faults that could cause unintended motion explicit handling. Keep blocking work out of the control loop.

## Controller implementation structure

- Keep controller hpp/cpp files focused on ROS lifecycle callbacks, parameter declaration/access, binding loaned interfaces, and exporting stable numeric storage.
- Put experiment configuration, input/output value types, validation, and control flow in a controller-specific logic header or source file. Use plain C++ values at this boundary so experiment logic can be read and tested without ROS or loaned interface types.
- Keep reusable mathematics in a small transformations/math file when it is shared or substantial. Use named samples and settings instead of mixing interface handles with measurements and configuration.
- Lifecycle callbacks should delegate domain decisions to the logic object. The update adapter should read a complete input snapshot, call the logic update, and publish its result. Perform configuration parsing and allocation before activation, and validate inputs before experiment calculations.
- When scaffolding an unfinished experiment, provide an explicit computation function with the required samples/settings available. Keep the unfinished output invalid until its author supplies the operation order and result. Do not invent the experiment's control policy.
- Test pure logic independently and use focused controller tests for interface binding, lifecycle, exports, and configuration handoff. Follow this pattern for new controllers without reorganizing unrelated existing packages.

## Tests and documentation

- Run the relevant existing checks. Add or extend a focused test when it covers an important behavior or regression the existing suite would miss. Keep one-off probes and debug output out of the commit.
- Before handoff, rebuild affected C/C++ targets and inspect compiler output for warnings. Fix project-source warnings and report unresolved upstream dependency warnings separately.
- Update the nearest existing documentation when a setup step, interface, or operating procedure changes. Record hard-to-reconstruct facts where they belong, such as the source of a calibration value or the reason for an unusual constraint.
- Summarize the change and the checks run in the final response. Keep task logs and handoff summaries in the conversation unless a durable record is requested.
