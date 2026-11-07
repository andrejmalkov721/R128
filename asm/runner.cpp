// R128 BIN loader + Verilator runner.
// Usage:
//   ./out/r128_runner path/to/program.bin
//   ./out/r128_runner path/to/program.bin --input 15 --latency 1
//   ./out/r128_runner path/to/program.bin --vcd out/custom.vcd --log out/custom.log
//
// BIN layout: raw byte image starting at physical address 0.
// R128 instructions are 16 bytes (128 bits), little-endian.

#include "Vmain.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Options {
    std::string bin;
    std::string vcd;
    std::string log;
    uint64_t input_value = 0;
    uint64_t max_cycles = 200000;
    int latency = 1;
};

// path helpers without C++17 filesystem
static std::string path_stem(const std::string& p) {
    size_t slash = p.find_last_of("/\\");
    std::string base = (slash == std::string::npos) ? p : p.substr(slash + 1);
    size_t dot = base.find_last_of('.');
    if (dot == std::string::npos || dot == 0) return base;
    return base.substr(0, dot);
}

static std::string path_parent(const std::string& p) {
    size_t slash = p.find_last_of("/\\");
    if (slash == std::string::npos) return ".";
    if (slash == 0) return "/";
    return p.substr(0, slash);
}

static std::string path_join(const std::string& dir, const std::string& name) {
    if (dir.empty() || dir == ".") return name;
    char last = dir.back();
    if (last == '/' || last == '\\') return dir + name;
    return dir + "/" + name;
}

static uint64_t parse_u64(const std::string& s) {
    size_t used = 0;
    uint64_t v = std::stoull(s, &used, 0);
    if (used != s.size()) throw std::runtime_error("invalid number: " + s);
    return v;
}

static Options parse_args(int argc, char** argv) {
    if (argc < 2) {
        throw std::runtime_error(
            "Usage: r128_runner <program.bin> [--input N] [--latency N] "
            "[--max-cycles N] [--vcd file.vcd] [--log file.log]");
    }

    Options o;
    o.bin = argv[1];
    const std::string stem = path_stem(o.bin);
    o.vcd = path_join(path_parent(o.bin), stem + ".vcd");
    o.log = path_join(path_parent(o.bin), stem + ".log");

    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        auto value = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value after " + a);
            return argv[++i];
        };
        if (a == "--input") o.input_value = parse_u64(value());
        else if (a == "--latency") o.latency = static_cast<int>(parse_u64(value()));
        else if (a == "--max-cycles") o.max_cycles = parse_u64(value());
        else if (a == "--vcd") o.vcd = value();
        else if (a == "--log") o.log = value();
        else throw std::runtime_error("unknown option: " + a);
    }
    if (o.latency < 0) throw std::runtime_error("--latency must be >= 0");
    return o;
}

static void write128(void* port, const std::array<uint8_t, 16>& bytes) {
    auto* words = static_cast<uint32_t*>(port);
    for (int i = 0; i < 4; ++i) {
        words[i] = uint32_t(bytes[4*i]) |
                   (uint32_t(bytes[4*i+1]) << 8) |
                   (uint32_t(bytes[4*i+2]) << 16) |
                   (uint32_t(bytes[4*i+3]) << 24);
    }
}

static std::array<uint8_t, 16> read128(const void* port) {
    const auto* words = static_cast<const uint32_t*>(port);
    std::array<uint8_t, 16> b{};
    for (int i = 0; i < 4; ++i) {
        b[4*i]   = uint8_t(words[i]);
        b[4*i+1] = uint8_t(words[i] >> 8);
        b[4*i+2] = uint8_t(words[i] >> 16);
        b[4*i+3] = uint8_t(words[i] >> 24);
    }
    return b;
}

static std::string hex128(const std::array<uint8_t, 16>& b) {
    std::ostringstream s;
    s << "0x";
    for (int i = 15; i >= 0; --i)
        s << std::hex << std::setw(2) << std::setfill('0') << unsigned(b[i]);
    return s.str();
}

class Runner {
public:
    explicit Runner(const Options& opts)
        : opt(opts), top(new Vmain), memory(1u << 20, 0) {
        std::ifstream f(opt.bin, std::ios::binary);
        if (!f) throw std::runtime_error("cannot open BIN: " + opt.bin);
        f.seekg(0, std::ios::end);
        const auto size = f.tellg();
        if (size <= 0) throw std::runtime_error("BIN is empty");
        if (static_cast<uint64_t>(size) > memory.size())
            throw std::runtime_error("BIN exceeds 1 MiB physical memory");
        f.seekg(0, std::ios::beg);
        f.read(reinterpret_cast<char*>(memory.data()), size);
        if (!f) throw std::runtime_error("failed to read BIN");
        image_size = static_cast<size_t>(size);

        log.open(opt.log);
        if (!log) throw std::runtime_error("cannot create LOG: " + opt.log);

        Verilated::traceEverOn(true);
        trace = new VerilatedVcdC;
        top->trace(trace, 99);
        trace->open(opt.vcd.c_str());
        if (!trace->isOpen()) throw std::runtime_error("cannot create VCD: " + opt.vcd);

        top->clk = 0;
        top->rst = 1;
        top->memReady = 0;
        std::array<uint8_t,16> zero{};
        write128(&top->memRData, zero);
        write128(&top->ioInData, zero);
    }

    ~Runner() {
        if (trace) {
            trace->close();
            delete trace;
        }
        if (top) {
            top->final();
            delete top;
        }
    }

    int run() {
        log << "R128 PROGRAM RUNNER\n"
            << "BIN: " << opt.bin << "\n"
            << "SIZE: " << image_size << " bytes\n"
            << "INPUT: " << opt.input_value << "\n"
            << "MEMORY LATENCY: " << opt.latency << "\n\n";
        std::cout << "R128: " << opt.bin << " (" << image_size << " bytes)\n";

        for (int i = 0; i < 4; ++i) step();
        top->rst = 0;

        bool halted = false;
        while (cycles < opt.max_cycles) {
            step();
            if (top->halt) {
                halted = true;
                break;
            }
        }

        log << "\n";
        log << "RESULT: " << (halted && protocol_errors == 0 ? "PASS" : "FAIL") << "\n";
        log << "HALT: " << (halted ? 1 : 0) << "\n";
        log << "CYCLES: " << cycles << "\n";
        log << "MEMORY TRANSACTIONS: " << transactions << "\n";
        log << "IO IN READS: " << input_reads << "\n";
        log << "IO OUT WRITES: " << output_writes << "\n";
        log << "PROTOCOL ERRORS: " << protocol_errors << "\n";
        log << "FINAL PC: 0x" << std::hex << uint64_t(top->pcOut) << std::dec << "\n";

        std::cout << (halted ? "HALT reached" : "TIMEOUT")
                  << " after " << cycles << " cycles; "
                  << "I/O reads=" << input_reads
                  << ", writes=" << output_writes
                  << ", protocol errors=" << protocol_errors << "\n";
        std::cout << "VCD: " << opt.vcd << "\nLOG: " << opt.log << "\n";
        return halted && protocol_errors == 0 ? 0 : 1;
    }

private:
    void dump(bool rising_edge) {
        trace->dump(static_cast<vluint64_t>(cycles * 2 + (rising_edge ? 1 : 0)));
    }

    void step() {
        top->clk = 0;
        top->eval();

        const bool req = top->memReq;
        const bool re = top->memRe;
        const bool we = top->memWe;
        const uint32_t addr = uint32_t(top->memAddr) & 0xFFFFF;
        const auto wdata = read128(&top->memWData);

        if (req != (re || we) || (req && (re == we)) || (!req && (re || we)))
            ++protocol_errors;

        if (!active && req) {
            active = true;
            pending_read = re;
            pending_write = we;
            pending_addr = addr;
            pending_data = wdata;
            countdown = opt.latency;
            ++transactions;
            log << "[MEM ISSUE] cycle=" << cycles
                << " " << (re ? "READ" : "WRITE")
                << " addr=0x" << std::hex << addr << std::dec << "\n";
        }

        top->memReady = 0;
        if (active) {
            // While waiting for latency, request must stay stable.
            // On the grant cycle the CPU may drop memReq — that is normal.
            if (countdown > 0) {
                if (!req || re != pending_read || we != pending_write ||
                    addr != pending_addr ||
                    (we && wdata != pending_data)) {
                    ++protocol_errors;
                }
                --countdown;
            } else {
                top->memReady = 1;
                if (pending_read) {
                    std::array<uint8_t,16> r{};
                    for (size_t i = 0; i < 16; ++i) {
                        const uint32_t a = (pending_addr + uint32_t(i)) & 0xFFFFF;
                        r[i] = memory[a];
                    }
                    write128(&top->memRData, r);
                }
                if (pending_write) {
                    for (size_t i = 0; i < 16; ++i) {
                        const uint32_t a = (pending_addr + uint32_t(i)) & 0xFFFFF;
                        memory[a] = pending_data[i];
                    }
                }
                log << "[MEM READY] cycle=" << cycles
                    << " addr=0x" << std::hex << pending_addr << std::dec << "\n";
                active = false;
            }
        }

        if (top->ioInRead) {
            std::array<uint8_t,16> din{};
            for (int i = 0; i < 8; ++i)
                din[i] = uint8_t(opt.input_value >> (8*i));
            write128(&top->ioInData, din);
            ++input_reads;
            log << "[IO IN] cycle=" << cycles
                << " port=" << unsigned(top->ioInPort)
                << " data=" << hex128(din) << "\n";
        }

        top->eval();
        dump(false);

        // ioOutWrite — строб до фронта; сохраняем его до изменения out_issued.
        const bool out_strobe = top->ioOutWrite;
        const uint8_t out_port = top->ioOutPort;
        const auto out_data = read128(&top->ioOutData);
        if (out_strobe) {
            ++output_writes;
            log << "[IO OUT] cycle=" << cycles
                << " port=" << unsigned(out_port)
                << " data=" << hex128(out_data) << "\n";
            std::cout << "OUT port " << unsigned(out_port)
                      << " = " << hex128(out_data) << "\n";
        }

        top->clk = 1;
        top->eval();
        dump(true);

        // memReady is a one-cycle grant; CPU may drop memReq after accepting.
        log << "[CYCLE] " << cycles
            << " PC=0x" << std::hex << uint64_t(top->pcOut)
            << std::dec << " HALT=" << unsigned(top->halt)
            << " MEMREQ=" << unsigned(top->memReq)
            << " IO_IN=" << unsigned(top->ioInRead)
            << " IO_OUT=" << unsigned(top->ioOutWrite) << "\n";
        ++cycles;
    }

    const Options& opt;
    Vmain* top = nullptr;
    VerilatedVcdC* trace = nullptr;
    std::ofstream log;
    std::vector<uint8_t> memory;
    size_t image_size = 0;
    uint64_t cycles = 0;
    uint64_t transactions = 0;
    uint64_t input_reads = 0;
    uint64_t output_writes = 0;
    uint64_t protocol_errors = 0;
    bool active = false;
    bool pending_read = false;
    bool pending_write = false;
    uint32_t pending_addr = 0;
    int countdown = 0;
    std::array<uint8_t,16> pending_data{};
};

int main(int argc, char** argv) {
    try {
        const Options opts = parse_args(argc, argv);
        Runner runner(opts);
        return runner.run();
    } catch (const std::exception& e) {
        std::cerr << "R128 runner error: " << e.what() << "\n";
        return 2;
    }
}
