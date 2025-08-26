use std::path::PathBuf;

fn main() {
    // Put the memory definition somewhere the linker can find it
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out_dir.display());
    std::fs::copy("memory-cb.x", out_dir.join("memory-cb.x")).unwrap();
    println!("cargo:rerun-if-changed=memory-cb.x");
}
