// Zig interop: dlopen the compiled Microduck walking kernel and call shinro_step once.
// Same port layout and same synthetic input as the C example (see interop/c/kernel_demo.c).
//
// The .so path comes from the MICRODUCK_SO env var (set by interop/Makefile),
// defaulting to ../build/compiled_policy/lib/lib_neural_network.so. Output goes
// through C's printf so the formatting matches the other languages byte for byte.
//
// Build:  zig build-exe -O ReleaseFast -lc -femit-bin=kernel_demo_zig kernel_demo.zig
const std = @import("std");

const N_IN = 61;
const N_OUT = 14;
const N_STATE = 1;

const ShinroStep = *const fn ([*]const f64, [*]f64, [*]f64) callconv(.c) void;

extern fn printf(fmt: [*:0]const u8, ...) c_int;
extern fn getenv(name: [*:0]const u8) ?[*:0]u8;

pub fn main() !void {
    const path = if (getenv("MICRODUCK_SO")) |p| std.mem.span(p) else "../build/compiled_policy/lib/lib_neural_network.so";

    var lib = try std.DynLib.open(path);
    defer lib.close();
    const step = lib.lookup(ShinroStep, "shinro_step") orelse return error.SymbolNotFound;

    var in: [N_IN]f64 = undefined;
    for (0..N_IN) |i| {
        in[i] = @as(f64, @floatFromInt(@as(i32, @intCast(i % 11)) - 5)) * 0.03125;
    }
    var out: [N_OUT]f64 = [_]f64{0} ** N_OUT;
    var state: [N_STATE]f64 = [_]f64{0} ** N_STATE;

    step(&in, &out, &state);
    _ = printf("%.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7], out[8], out[9], out[10], out[11], out[12], out[13]);
}
