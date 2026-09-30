// Rust interop: dlopen the compiled Microduck walking kernel and call shinro_step once.
// Same port layout and same synthetic input as the C example (interop/c/kernel_demo.c).
//
// Built with `rustc` alone — no cargo, no crates. std has no dynamic-loader
// wrapper, so `dlopen`/`dlsym` are declared through the C ABI directly; the kernel
// is plain C, so nothing more is needed. Output goes through C's printf so the
// rendering matches the other languages byte for byte.
//
// Build:  rustc -O -o kernel_demo_rust kernel_demo.rs -l dl
// Run:    ./kernel_demo_rust /path/to/lib_neural_network.so

use std::ffi::{c_char, c_double, c_int, c_void, CString};
use std::process::exit;

const N_IN: usize = 61;
const N_OUT: usize = 14;
const N_STATE: usize = 1;
const RTLD_NOW: c_int = 2;
const DEFAULT_SO: &str = "../build/compiled_policy/lib/lib_neural_network.so";

extern "C" {
    fn dlopen(filename: *const c_char, flags: c_int) -> *mut c_void;
    fn dlsym(handle: *mut c_void, symbol: *const c_char) -> *mut c_void;
    // C printf, so `%.17g` matches the C/C++/Zig/Python hosts exactly.
    fn printf(fmt: *const c_char, ...) -> c_int;
}

type ShinroStep = unsafe extern "C" fn(*const c_double, *mut c_double, *mut c_double);

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| DEFAULT_SO.to_string());
    let c_path = CString::new(path).expect("path contains a NUL byte");

    unsafe {
        let lib = dlopen(c_path.as_ptr(), RTLD_NOW);
        if lib.is_null() {
            eprintln!("dlopen failed");
            exit(1);
        }
        let sym = dlsym(lib, b"shinro_step\0".as_ptr() as *const c_char);
        if sym.is_null() {
            eprintln!("dlsym failed");
            exit(1);
        }
        let step: ShinroStep = std::mem::transmute(sym);

        // The shared synthetic observation: slot i -> ((i % 11) - 5) / 32, exactly
        // representable, so every language feeds bit-identical inputs.
        let mut inp = [0.0f64; N_IN];
        for (i, value) in inp.iter_mut().enumerate() {
            *value = ((i % 11) as i64 - 5) as f64 * 0.03125;
        }
        let mut out = [0.0f64; N_OUT];
        let mut state = [0.0f64; N_STATE];

        step(inp.as_ptr(), out.as_mut_ptr(), state.as_mut_ptr());

        let fmt = b"%.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n\0";
        printf(
            fmt.as_ptr() as *const c_char,
            out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7], out[8], out[9], out[10],
            out[11], out[12], out[13],
        );
    }
}
