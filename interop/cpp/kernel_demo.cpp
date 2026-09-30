// C++ interop: dlopen the compiled Microduck walking kernel and call shinro_step once.
// Same port layout and same synthetic input as the C example (see interop/c/kernel_demo.c).
//
// Build:  c++ -std=c++17 -O2 -o kernel_demo_cpp kernel_demo.cpp -ldl
// Run:    ./kernel_demo_cpp /path/to/lib_neural_network.so
#include <dlfcn.h>

#include <array>
#include <cstdio>

namespace {
constexpr int N_IN = 61;
constexpr int N_OUT = 14;
constexpr int N_STATE = 1;
using ShinroStep = void (*)(const double *, double *, double *);

// slot i -> ((i % 11) - 5) / 32, exactly representable in every language.
void fill_input(std::array<double, N_IN> &in) {
    for (int i = 0; i < N_IN; ++i) {
        in[i] = static_cast<double>((i % 11) - 5) * 0.03125;
    }
}
}  // namespace

int main(int argc, char **argv) {
    const char *path = argc > 1 ? argv[1] : "../build/compiled_policy/lib/lib_neural_network.so";

    void *lib = dlopen(path, RTLD_NOW);
    if (!lib) {
        std::fprintf(stderr, "dlopen failed: %s\n", dlerror());
        return 1;
    }
    auto step = reinterpret_cast<ShinroStep>(dlsym(lib, "shinro_step"));
    if (!step) {
        std::fprintf(stderr, "dlsym failed: %s\n", dlerror());
        return 1;
    }

    std::array<double, N_IN> in{};
    std::array<double, N_OUT> out{};
    std::array<double, N_STATE> state{};
    fill_input(in);

    step(in.data(), out.data(), state.data());
    for (int i = 0; i < N_OUT; ++i) {
        std::printf("%.17g%c", out[i], i + 1 < N_OUT ? ' ' : '\n');
    }

    dlclose(lib);
    return 0;
}
