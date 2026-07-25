fn main() {
    println!("cargo:rerun-if-changed=native/cvi_tpu.c");
    println!("cargo:rerun-if-changed=native/cvi_tpu.h");
    println!("cargo:rerun-if-env-changed=CVI_RUNTIME_INCLUDE");
    println!("cargo:rerun-if-env-changed=CVI_RUNTIME_LIB_DIR");
    println!("cargo:rerun-if-changed=native/cvi_camera.c");
    println!("cargo:rerun-if-changed=native/cvi_camera.h");
    println!("cargo:rerun-if-env-changed=CVI_MPI_INCLUDE");
    println!("cargo:rerun-if-env-changed=CVI_MPI_ROOT");
    println!("cargo:rerun-if-env-changed=CVI_MPI_SAMPLE_INCLUDE");
    println!("cargo:rerun-if-env-changed=CVI_MPI_ISP_INCLUDE");
    println!("cargo:rerun-if-env-changed=CVI_MPI_COMMON_INCLUDE");
    println!("cargo:rerun-if-env-changed=CVI_MPI_LIB_DIR");
    println!("cargo:rerun-if-env-changed=CVI_TOOLCHAIN_ATOMIC_LIB_DIR");

    // The normal PC build deliberately has no dependency on the vendor SDK.
    if std::env::var_os("CARGO_FEATURE_CVI_RUNTIME").is_none() {
        return;
    }

    let include = std::env::var("CVI_RUNTIME_INCLUDE")
        .expect("CVI_RUNTIME_INCLUDE must point to the directory containing cviruntime.h");
    cc::Build::new()
        .file("native/cvi_tpu.c")
        .include(include)
        .warnings(true)
        .compile("rubik_cvi_tpu");

    if let Ok(lib_dir) = std::env::var("CVI_RUNTIME_LIB_DIR") {
        println!("cargo:rustc-link-search=native={lib_dir}");
    }
    println!("cargo:rustc-link-lib=dylib=cviruntime");
    println!("cargo:rustc-link-lib=dylib=cvikernel");
    // cviruntime is implemented in C++ and advertises libstdc++.so.6 as a
    // runtime dependency. Linking it explicitly keeps the final executable's
    // dependency set unambiguous when Cargo invokes the C linker.
    println!("cargo:rustc-link-lib=dylib=stdc++");
    println!("cargo:rustc-link-lib=dylib=dl");

    if std::env::var_os("CARGO_FEATURE_CVI_CAMERA").is_none() {
        return;
    }

    let mpi_include =
        std::env::var("CVI_MPI_INCLUDE").expect("CVI_MPI_INCLUDE must point to cvi_mpi/include");
    let mpi_root =
        std::env::var("CVI_MPI_ROOT").expect("CVI_MPI_ROOT must point to cvi_mpi source");
    let sample_include = std::env::var("CVI_MPI_SAMPLE_INCLUDE")
        .expect("CVI_MPI_SAMPLE_INCLUDE must point to cvi_mpi/sample/common");
    let isp_include = std::env::var("CVI_MPI_ISP_INCLUDE")
        .expect("CVI_MPI_ISP_INCLUDE must point to CV181X ISP headers");
    let common_include = std::env::var("CVI_MPI_COMMON_INCLUDE")
        .expect("CVI_MPI_COMMON_INCLUDE must point to cvi_mpi/component/isp/common");
    cc::Build::new()
        .file("native/cvi_camera.c")
        // Avoid monolithic libsample.so. These are the source units reached by
        // the VI/ISP lifecycle, with GC2083 as the only sensor backend.
        .file(format!("{mpi_root}/sample/common/sample_common_sys.c"))
        .file(format!("{mpi_root}/sample/common/sample_common_platform.c"))
        .file(format!("{mpi_root}/sample/common/sample_common_vi.c"))
        .file(format!("{mpi_root}/sample/common/sample_common_isp.c"))
        .file(format!("{mpi_root}/sample/common/sample_common_sensor.c"))
        .file(format!("{mpi_root}/sample/common/sample_common_bin.c"))
        .file(format!("{mpi_root}/component/isp/common/sensor_list.c"))
        .include(mpi_include)
        .include(sample_include)
        .include(isp_include)
        .include(common_include)
        .include(format!("{mpi_root}/3rdparty/inih"))
        .define("__CV181X__", None)
        .define("OS_IS_LINUX", None)
        .define("SENSOR_GCORE_GC2083", None)
        .warnings(true)
        .compile("rubik_cvi_camera");

    let mpi_lib_dir =
        std::env::var("CVI_MPI_LIB_DIR").expect("CVI_MPI_LIB_DIR must point to cvi_mpi/lib");
    println!("cargo:rustc-link-search=native={mpi_lib_dir}");
    for lib in [
        "vi",
        "isp",
        "sys",
        "vpss",
        "vo",
        "gdc",
        "cvi_bin",
        "cvi_bin_isp",
        "af",
        "ae",
        "awb",
        "isp_algo",
        "sns_gc2083",
    ] {
        println!("cargo:rustc-link-lib=dylib={lib}");
    }
    println!("cargo:rustc-link-search=native={mpi_lib_dir}/3rd");
    println!("cargo:rustc-link-lib=static=ini");
    println!("cargo:rustc-link-lib=dylib=pthread");
    // The board image does not provide libatomic.so; include the tiny GCC
    // atomic runtime in the executable for libsys' byte-CAS helper.
    let atomic_lib_dir = std::env::var("CVI_TOOLCHAIN_ATOMIC_LIB_DIR")
        .expect("CVI_TOOLCHAIN_ATOMIC_LIB_DIR must point to the target libatomic.a directory");
    println!("cargo:rustc-link-search=native={atomic_lib_dir}");
    println!("cargo:rustc-link-lib=static=atomic");
}
