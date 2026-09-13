//! Regenerate `include/mfsk_ft8.h` from the FFI surface on every build.
//!
//! Same two rules as `mfsk-ffi/build.rs`, for the same reasons: a cbindgen
//! failure is fatal rather than a `cargo:warning` that ships a stale
//! committed header, and `mfsk-ffi-abi` — which `cbindgen.toml` pulls in
//! via `parse_deps` — is a rerun trigger.

use std::env;
use std::path::PathBuf;

fn main() {
    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_path = crate_dir.join("include").join("mfsk_ft8.h");
    if let Some(parent) = out_path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }

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
    println!("cargo:rerun-if-changed=../mfsk-ffi-abi/src/lib.rs");
    println!("cargo:rerun-if-changed=../mfsk-ffi-abi/Cargo.toml");
}
