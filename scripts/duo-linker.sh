#!/usr/bin/env bash
# Cargo's linker for the RISC-V Duo target. The SDK GCC driver still supplies
# the target musl sysroot; only GNU ld is replaced with modern ld.lld.
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
sdk_root="$project_root/duo-buildroot-sdk-v2"
gcc="$sdk_root/host-tools/gcc/riscv64-linux-musl-x86_64/bin/riscv64-unknown-linux-musl-gcc"
lld_shim_dir=${RUBIK_DUO_LLD_DIR:-"$project_root/target/duo-lld"}

# Some vendor .so files (notably libcvi_ispd2) deliberately leave optional
# JSON/bin hooks unresolved. This matches the SDK sample link mode; unresolved
# references in our own objects are still rejected normally.
exec "$gcc" -B"$lld_shim_dir" -fuse-ld=lld "$@" -Wl,--allow-shlib-undefined
