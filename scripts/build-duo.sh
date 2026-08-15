#!/usr/bin/env bash
# Build rubik-scan for Milk-V Duo 256M (SG2002, RISC-V musl).
#
# Prerequisite: build_3rd_party + build_tpu_sdk have completed inside the
# duo-buildroot-sdk-v2 submodule. See PROJECT_NOTES.md, section 10.13.
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
sdk_root="$project_root/duo-buildroot-sdk-v2"
toolchain_root="$sdk_root/host-tools/gcc/riscv64-linux-musl-x86_64"
tpu_sdk_root="$sdk_root/install/soc_sg2002_milkv_duo256m_musl_riscv64_sd/tpu_musl_riscv64/cvitek_tpu_sdk"
mpi_root="$sdk_root/cvi_mpi"
target="riscv64gc-unknown-linux-musl"
cc="$toolchain_root/bin/riscv64-unknown-linux-musl-gcc"
lld=$(command -v ld.lld || true)

for required in "$cc" "$tpu_sdk_root/include/cviruntime.h" \
                "$tpu_sdk_root/lib/libcviruntime.so" "$tpu_sdk_root/lib/libcvikernel.so" \
                "$mpi_root/include/cvi_vi.h" "$mpi_root/sample/common/sample_comm.h" \
                "$mpi_root/lib/libcvi_bin.so" "$mpi_root/lib/libcvi_bin_isp.so" \
                "$mpi_root/lib/libsns_gc2083.so"; do
  if [[ ! -e "$required" ]]; then
    echo "Missing prerequisite: $required" >&2
    echo "Build the Duo TPU SDK first; see PROJECT_NOTES.md section 10.13." >&2
    exit 1
  fi
done
if [[ -z "$lld" ]]; then
  echo "Missing ld.lld; install it with: sudo apt install lld" >&2
  exit 1
fi

# The SDK GNU ld is older than the RISC-V ISA attributes emitted by current
# Rust. Keep the SDK GCC driver (musl sysroot/CRT/ABI), but make its
# `-fuse-ld=lld` lookup resolve to the host's modern LLVM linker.
lld_shim_dir="$project_root/target/duo-lld"
mkdir -p "$lld_shim_dir"
ln -sfn "$lld" "$lld_shim_dir/ld.lld"

export CC_riscv64gc_unknown_linux_musl="$cc"
export CARGO_TARGET_RISCV64GC_UNKNOWN_LINUX_MUSL_LINKER="$project_root/scripts/duo-linker.sh"
export CVI_RUNTIME_INCLUDE="$tpu_sdk_root/include"
export CVI_RUNTIME_LIB_DIR="$tpu_sdk_root/lib"
export CVI_MPI_INCLUDE="$mpi_root/include"
export CVI_MPI_ROOT="$mpi_root"
export CVI_MPI_SAMPLE_INCLUDE="$mpi_root/sample/common"
export CVI_MPI_ISP_INCLUDE="$mpi_root/include/isp/cv181x"
export CVI_MPI_COMMON_INCLUDE="$mpi_root/component/isp/common"
export CVI_MPI_LIB_DIR="$mpi_root/lib"
# GCC selects this exact multilib for riscv64gc / the Duo's Xthead ABI. Cargo
# needs it explicitly when linking the package library target as well as bins.
export CVI_TOOLCHAIN_ATOMIC_LIB_DIR="$toolchain_root/sysroot/lib64xthead/lp64d"
export RUBIK_DUO_LLD_DIR="$lld_shim_dir"

cd "$project_root"
exec cargo build --release --target "$target" --features cvi-camera,pca9685 \
  --bin rubik-scan --bin rubik-camera-probe --bin rubik-solve \
  --bin rubik-servo-probe --bin rubik-servo-init --bin rubik-servo-calibrate \
  --bin rubik-stand --bin rubik-stand-runtime "$@"
