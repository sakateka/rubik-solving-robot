#!/usr/bin/env bash
# Cargo's linker for the RISC-V Duo target. The SDK GCC driver still supplies
# the target musl sysroot; only GNU ld is replaced with modern ld.lld.
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
sdk_root="$project_root/duo-buildroot-sdk-v2"
gcc="$sdk_root/host-tools/gcc/riscv64-linux-musl-x86_64/bin/riscv64-unknown-linux-musl-gcc"
lld_shim_dir=${RUBIK_DUO_LLD_DIR:-"$project_root/target/duo-lld"}

# Rust injects --as-needed before native libraries. CVI's libraries do not
# reliably declare their own DT_NEEDED edges, so retain the media libraries
# Cargo names on the executable instead of letting LLD discard them.
link_args=()
for arg in "$@"; do
  if [[ "$arg" == "-Wl,--as-needed" ]]; then
    link_args+=("-Wl,--no-as-needed")
  else
    link_args+=("$arg")
  fi
done

exec "$gcc" -B"$lld_shim_dir" -fuse-ld=lld "${link_args[@]}" -Wl,--allow-shlib-undefined
