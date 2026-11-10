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

## Tests and documentation

- Run the relevant existing checks. Add or extend a focused test when it covers an important behavior or regression the existing suite would miss. Keep one-off probes and debug output out of the commit.
- Update the nearest existing documentation when a setup step, interface, or operating procedure changes. Record hard-to-reconstruct facts where they belong, such as the source of a calibration value or the reason for an unusual constraint.
- Summarize the change and the checks run in the final response. Keep task logs and handoff summaries in the conversation unless a durable record is requested.
