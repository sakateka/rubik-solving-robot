# Contributing

## Unsafe Rust and FFI

Keep `unsafe` scopes as small as possible.

- Safe application code must not manipulate raw pointers or call the CVI C API.
- Confine FFI calls to a narrow adapter or wrapper that exposes a safe Rust API.
- Each `unsafe` block needs a `SAFETY:` comment describing the concrete
  lifetime, ownership, alignment, and ABI invariants it relies on.
- Prefer an owning RAII type (`Drop`) over manual open/close pairs.
- Do not make an entire function `unsafe` when one expression needs it.
- Adding or expanding `unsafe` requires a review of the C contract and a
  relevant test or hardware validation step.
