// The soname carries FLUX_ABI_VERSION, so a binary linked against one ABI never loads another.
fn main() {
    println!("cargo:rerun-if-changed=src/lib.rs");
    let lib = std::fs::read_to_string("src/lib.rs").expect("src/lib.rs");
    let abi = lib
        .lines()
        .find_map(|l| l.strip_prefix("pub const FLUX_ABI_VERSION: u32 = "))
        .and_then(|v| v.strip_suffix(';'))
        .expect("FLUX_ABI_VERSION in src/lib.rs");
    println!("cargo:rustc-cdylib-link-arg=-Wl,-soname,libflux_ffi.so.{abi}");
}
