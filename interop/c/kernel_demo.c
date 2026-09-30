/* C-ABI interop: dlopen the compiled Microduck walking kernel and call shinro_step once.
 *
 * The policy kernel is a pure function of its ports:
 *     void shinro_step(const double* in, double* out, double* state);
 * with the layout in build/compiled_policy/graph_data_manifest.json:
 *     in    = state(61)   the raw 61-D observation  -> 61
 *     out   = u(14)       joint-position offsets    -> 14
 *     state = (empty)     memoryless MLP: no recurrent ports; pointer is scratch
 *
 * This file has no dependency beyond libc + libdl: no Python, no shinro, no
 * numpy. That is the artifact's whole claim. The same 61-D input is rebuilt
 * identically in the C++, Zig and Python siblings; all four must print the
 * same 14-D action.
 *
 * Build:  cc -O2 -o kernel_demo_c kernel_demo.c -ldl
 * Run:    ./kernel_demo_c /path/to/lib_neural_network.so
 */
#include <dlfcn.h>
#include <stdio.h>

#define N_IN 61
#define N_OUT 14
#define N_STATE 1

typedef void (*shinro_step_fn)(const double *, double *, double *);

/* The shared synthetic observation. Every value is exactly representable, so all
 * four languages feed bit-identical inputs: slot i -> ((i % 11) - 5) / 32. */
static void fill_input(double *in, int n) {
    for (int i = 0; i < n; ++i) {
        in[i] = (double)((i % 11) - 5) * 0.03125;
    }
}

int main(int argc, char **argv) {
    const char *path = argc > 1 ? argv[1] : "../build/compiled_policy/lib/lib_neural_network.so";

    void *lib = dlopen(path, RTLD_NOW);
    if (!lib) {
        fprintf(stderr, "dlopen failed: %s\n", dlerror());
        return 1;
    }
    shinro_step_fn step = (shinro_step_fn)dlsym(lib, "shinro_step");
    if (!step) {
        fprintf(stderr, "dlsym failed: %s\n", dlerror());
        return 1;
    }

    double in[N_IN], out[N_OUT] = {0}, state[N_STATE] = {0};
    fill_input(in, N_IN);

    step(in, out, state);
    for (int i = 0; i < N_OUT; ++i) {
        printf("%.17g%c", out[i], i + 1 < N_OUT ? ' ' : '\n');
    }

    dlclose(lib);
    return 0;
}
