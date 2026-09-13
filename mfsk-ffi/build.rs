//! Regenerate `include/mfsk.h` from the FFI surface on every build.
//!
//! Two things here are load-bearing and were not before.
//!
//! **A cbindgen failure is fatal.** It used to be `cargo:warning=`, which
//! meant a build that could not regenerate the header still succeeded and
//! shipped whatever stale copy was committed. The header is the ABI as far
//! as every C, Kotlin and Swift consumer is concerned; failing to produce
//! it is not a warning.
//!
//! **`mfsk-ffi-abi` is a rerun trigger.** `cbindgen.toml` sets
//! `parse_deps = true` with `include = ["mfsk-ffi-abi"]`, so the shared
//! `#[repr(C)]` types are pulled across the crate boundary into this
//! header — but cargo was never told, so editing `MfskResult` left the
//! committed header stale with nothing to notice.

use std::env;
use std::path::PathBuf;

fn main() {
    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_path = crate_dir.join("include").join("mfsk.h");

    match cbindgen::generate(&crate_dir) {
        Ok(bindings) => {
            bindings.write_to_file(&out_path);
        }
        Err(e) => {
            panic!("cbindgen failed to generate {}: {e}", out_path.display());
        }
    }

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=cbindgen.toml");
    println!("cargo:rerun-if-changed=build.rs");
    // The types `parse_deps` reaches for. Without this, a change there
    // silently leaves `include/mfsk.h` describing the previous ABI.
    println!("cargo:rerun-if-changed=../mfsk-ffi-abi/src/lib.rs");
    println!("cargo:rerun-if-changed=../mfsk-ffi-abi/Cargo.toml");
}
