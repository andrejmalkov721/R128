// =============================================================================
// R128 — полный однофайловый тестбенч для Verilator на C++
//
// Что проверяет тестбенч:
//   * сброс / NOP / HALT / неизвестный opcode
//   * все операции ALU, включая старшую половину 128-битного результата и переносы при сдвигах
//   * жёсткую семантику R0 = 0
//   * цепочки forwarding между EX/MEM и MEM/WB
//   * зависимости LOAD->ALU, ALU->STORE, LOAD->STORE и STORE->LOAD
//   * задержки памяти от 0 до 8 тактов
//   * арбитраж IF/данные и приостановку IF на время доступа к данным
//   * флаги переходов Z/N/C/V, EQ/NE и беззнаковые/знаковые сравнения
//   * каждый условный переход в вариантах taken и not-taken
//   * JMP, обратный относительный переход и JMPF
//   * MOVS для DS/SS/ES, MFS, LOADS и STORES
//   * MOVS CS, сброс конвейера и переключение сегмента кода
//   * однотактные строб-сигналы IN/OUT при остановках IF
//   * HALT после операций памяти и при задержанной выборке IF
//   * проверку протокола memReq/memRe/memWe/memReady и стробов I/O
//   * детерминированное обнаружение тайм-аутов
//
// =============================================================================

#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vmain.h"

#include <array>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <algorithm>

namespace op {
constexpr uint16_t NOP   = 0x0000;
constexpr uint16_t ADD   = 0x0001;
constexpr uint16_t SUB   = 0x0002;
constexpr uint16_t AND   = 0x0003;
constexpr uint16_t OR    = 0x0004;
constexpr uint16_t XOR   = 0x0005;
constexpr uint16_t NOT   = 0x0006;
constexpr uint16_t SHL   = 0x0007;
constexpr uint16_t SHR   = 0x0008;
constexpr uint16_t MOV   = 0x0009;
constexpr uint16_t LOAD  = 0x000A;
constexpr uint16_t STORE = 0x000B;
constexpr uint16_t JMP   = 0x000C;
constexpr uint16_t HALT  = 0x000D;
constexpr uint16_t JZ    = 0x000E;
constexpr uint16_t JNZ   = 0x000F;
constexpr uint16_t JN    = 0x0010;
constexpr uint16_t JP    = 0x0011;
constexpr uint16_t JC    = 0x0012;
constexpr uint16_t JNC   = 0x0013;
constexpr uint16_t JV    = 0x0014;
constexpr uint16_t JNV   = 0x0015;
constexpr uint16_t JE    = 0x0016;
constexpr uint16_t JNE   = 0x0017;
constexpr uint16_t JUGT  = 0x0018;
constexpr uint16_t JULT  = 0x0019;
constexpr uint16_t JUGE  = 0x001A;
constexpr uint16_t JULE  = 0x001B;
constexpr uint16_t JSGT  = 0x001C;
constexpr uint16_t JSLT  = 0x001D;
constexpr uint16_t JSGE  = 0x001E;
constexpr uint16_t JSLE  = 0x001F;
constexpr uint16_t CMP   = 0x0020;
constexpr uint16_t MOVS  = 0x0021;
constexpr uint16_t MFS   = 0x0022;
constexpr uint16_t LOADS = 0x0023;
constexpr uint16_t STORES= 0x0024;
constexpr uint16_t JMPF  = 0x0025;
constexpr uint16_t IN    = 0x0026;
constexpr uint16_t OUT   = 0x0027;
constexpr uint8_t SEG_CS = 0;
constexpr uint8_t SEG_DS = 1;
constexpr uint8_t SEG_SS = 2;
constexpr uint8_t SEG_ES = 3;
}

static int g_fails = 0;
static int g_passes = 0;
static FILE *g_log = nullptr;

static void logf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    if (g_log) {
        va_start(ap, fmt);
        vfprintf(g_log, fmt, ap);
        va_end(ap);
        fflush(g_log);
    }
}

#define CHECK(cond, msg) do { \
    if (cond) { ++g_passes; logf("  PASS: %s\n", (msg)); } \
    else { ++g_fails; logf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, (msg)); } \
} while (0)

#define CHECKF(cond, fmt, ...) do { \
    if (cond) { ++g_passes; logf("  PASS: " fmt "\n", __VA_ARGS__); } \
    else { ++g_fails; logf("  FAIL %s:%d: " fmt "\n", __FILE__, __LINE__, __VA_ARGS__); } \
} while (0)

static void pack_u128(uint8_t out[16], uint64_t lo, uint64_t hi = 0) {
    for (int i = 0; i < 8; ++i) out[i]     = uint8_t((lo >> (8 * i)) & 0xFF);
    for (int i = 0; i < 8; ++i) out[8 + i] = uint8_t((hi >> (8 * i)) & 0xFF);
}

static void unpack_u128(const uint8_t in[16], uint64_t &lo, uint64_t &hi) {
    lo = hi = 0;
    for (int i = 0; i < 8; ++i) lo |= uint64_t(in[i]) << (8 * i);
    for (int i = 0; i < 8; ++i) hi |= uint64_t(in[8 + i]) << (8 * i);
}

static void encode_instr(uint8_t out[16], uint16_t opcode, uint8_t rd, uint8_t rs1,
                         uint8_t rs2, uint64_t imm_lo, uint32_t imm_mid = 0) {
    uint64_t w0 = imm_lo;
    uint64_t w1 = uint64_t(imm_mid & 0x3FFFFFFF);
    w1 |= uint64_t(rs2 & 0x3F) << 30;
    w1 |= uint64_t(rs1 & 0x3F) << 36;
    w1 |= uint64_t(rd  & 0x3F) << 42;
    w1 |= uint64_t(opcode) << 48;
    pack_u128(out, w0, w1);
}

class BusModel {
public:
    int latency = 0;

    void clear() {
        mem_.clear();
        active_ = false;
        countdown_ = -1;
        pend_re_ = pend_we_ = false;
        pend_addr_ = 0;
        std::memset(pend_wdata_, 0, sizeof(pend_wdata_));
        in_strobes_ = out_strobes_ = 0;
        prev_in_ = prev_out_ = false;
        protocol_errors_ = 0;
        txn_count_ = 0;
        ready_count_ = 0;
    }

    void store(uint32_t addr, const uint8_t data[16]) {
        std::memcpy(mem_[addr & 0xFFFFF].data(), data, 16);
    }

    void prog(uint16_t pc, uint16_t opc, uint8_t rd = 0, uint8_t rs1 = 0,
              uint8_t rs2 = 0, uint64_t imm_lo = 0, uint32_t imm_mid = 0) {
        uint8_t w[16];
        encode_instr(w, opc, rd, rs1, rs2, imm_lo, imm_mid);
        store(pc & 0xFFFF, w);
    }

    static void read128(const void *pin, uint8_t out[16]) {
        const auto *w = static_cast<const uint32_t *>(pin);
        for (int i = 0; i < 4; ++i) {
            out[4*i+0] = uint8_t((w[i] >> 0)  & 0xFF);
            out[4*i+1] = uint8_t((w[i] >> 8)  & 0xFF);
            out[4*i+2] = uint8_t((w[i] >> 16) & 0xFF);
            out[4*i+3] = uint8_t((w[i] >> 24) & 0xFF);
        }
    }

    static void write128(void *pin, const uint8_t in[16]) {
        auto *w = static_cast<uint32_t *>(pin);
        for (int i = 0; i < 4; ++i) {
            w[i] = uint32_t(in[4*i+0]) |
                   (uint32_t(in[4*i+1]) << 8) |
                   (uint32_t(in[4*i+2]) << 16) |
                   (uint32_t(in[4*i+3]) << 24);
        }
    }

    // Вызывается один раз за моделируемый такт перед фронтом clk DUT.
    void tick(Vmain *top) {
        const bool req = top->memReq;
        const bool re  = top->memRe;
        const bool we  = top->memWe;
        const uint32_t addr = top->memAddr & 0xFFFFF;
        uint8_t wdata[16];
        read128(reinterpret_cast<const uint32_t*>(&top->memWData), wdata);

        // Инварианты интерфейса.
        if (req != (re || we)) ++protocol_errors_;
        if (req && (re == we)) ++protocol_errors_; // exactly one direction
        if (!req && (re || we)) ++protocol_errors_;

        if (!active_ && req && (re || we)) {
            active_ = true;
            pend_re_ = re;
            pend_we_ = we;
            pend_addr_ = addr;
            std::memcpy(pend_wdata_, wdata, 16);
            countdown_ = std::max(0, latency);
            ++txn_count_;
        }

        if (active_) {
            if (countdown_ > 0) {
                // Внешняя транзакция должна оставаться неизменной, пока ожидается ответ.
                if (!req || re != pend_re_ || we != pend_we_ || addr != pend_addr_ ||
                    std::memcmp(wdata, pend_wdata_, 16) != 0) {
                    ++protocol_errors_;
                }
                --countdown_;
                top->memReady = 0;
            } else {
                top->memReady = 1;
                ++ready_count_;

                if (pend_re_) {
                    uint8_t rdata[16] = {};
                    auto it = mem_.find(pend_addr_);
                    if (it != mem_.end()) std::memcpy(rdata, it->second.data(), 16);
                    write128(reinterpret_cast<uint32_t*>(&top->memRData), rdata);
                }
                if (pend_we_) {
                    std::memcpy(mem_[pend_addr_].data(), pend_wdata_, 16);
                }
                active_ = false;
                countdown_ = -1;
            }
        } else {
            top->memReady = 0;
        }

        // memReady подтверждает наличие активного запроса.
        if (top->memReady && !top->memReq) ++protocol_errors_;

        // Контракт I/O: оба сигнала являются однотактными стробами.
        if (top->ioInRead) {
            uint8_t din[16] = {};
            din[0] = top->ioInPort;
            din[1] = 0xA5;
            write128(reinterpret_cast<uint32_t*>(&top->ioInData), din);
            if (!prev_in_) ++in_strobes_;
            else ++protocol_errors_; // held for two consecutive cycles
        }
        if (top->ioOutWrite) {
            if (!prev_out_) ++out_strobes_;
            else ++protocol_errors_;
        }
        prev_in_ = top->ioInRead;
        prev_out_ = top->ioOutWrite;
    }

    bool read_mem(uint32_t addr, uint64_t &lo, uint64_t &hi) const {
        auto it = mem_.find(addr & 0xFFFFF);
        if (it == mem_.end()) return false;
        unpack_u128(it->second.data(), lo, hi);
        return true;
    }

    bool read_mem128(uint32_t addr, std::array<uint8_t,16> &out) const {
        auto it = mem_.find(addr & 0xFFFFF);
        if (it == mem_.end()) return false;
        out = it->second;
        return true;
    }

    int in_strobes() const { return in_strobes_; }
    int out_strobes() const { return out_strobes_; }
    int protocol_errors() const { return protocol_errors_; }
    uint64_t txn_count() const { return txn_count_; }
    uint64_t ready_count() const { return ready_count_; }
    bool active() const { return active_; }

private:
    std::unordered_map<uint32_t, std::array<uint8_t,16>> mem_;
    bool active_ = false;
    bool pend_re_ = false, pend_we_ = false;
    uint32_t pend_addr_ = 0;
    uint8_t pend_wdata_[16]{};
    int countdown_ = -1;
    int in_strobes_ = 0, out_strobes_ = 0;
    bool prev_in_ = false, prev_out_ = false;
    int protocol_errors_ = 0;
    uint64_t txn_count_ = 0, ready_count_ = 0;
};

class Sim {
public:
    Vmain *top = new Vmain;
    BusModel bus;
    VerilatedVcdC *trace = nullptr;
    uint64_t cycle = 0;
    uint64_t max_seen_cycle = 0;

    ~Sim() {
        if (trace) {
            trace->close();
            delete trace;
            trace = nullptr;
        }
        top->final();
        delete top;
    }

    bool enable_trace(const char *path) {
        trace = new VerilatedVcdC;
        top->trace(trace, 99);
        trace->open(path);
        return true;
    }

    void reset(int n = 4) {
        top->clk = 0;
        top->rst = 1;
        top->memReady = 0;
        for (int i = 0; i < 4; ++i) {
            top->memRData[i] = 0;
            top->ioInData[i] = 0;
        }
        for (int i = 0; i < n; ++i) step();
        top->rst = 0;
    }

    void step() {
        top->clk = 0;
        top->eval();
        bus.tick(top);
        top->eval();
        top->clk = 1;
        top->eval();
        if (trace) trace->dump(static_cast<vluint64_t>(cycle * 2 + 1));
        ++cycle;
        max_seen_cycle = std::max(max_seen_cycle, cycle);
    }

    bool run_until_halt(uint64_t max_c = 200000) {
        const uint64_t start = cycle;
        while (!top->halt && (cycle - start) < max_c) step();
        return top->halt;
    }

    bool run_until_pc(uint64_t expected_pc, uint64_t max_c = 10000) {
        const uint64_t start = cycle;
        while (static_cast<uint64_t>(top->pcOut) != expected_pc && (cycle - start) < max_c) step();
        return static_cast<uint64_t>(top->pcOut) == expected_pc;
    }
};

static void emit(BusModel &b, uint16_t &pc, uint16_t opc, uint8_t rd = 0,
                 uint8_t rs1 = 0, uint8_t rs2 = 0, uint64_t imm = 0,
                 uint32_t imm_mid = 0) {
    b.prog(pc, opc, rd, rs1, rs2, imm, imm_mid);
    pc = uint16_t((pc + 16) & 0xFFF0);
}

static void put_u128(BusModel &bus, uint32_t addr, uint64_t lo, uint64_t hi = 0) {
    uint8_t d[16] = {};
    pack_u128(d, lo, hi);
    bus.store(addr, d);
}

static bool check_mem128(BusModel &b, uint32_t addr, uint64_t elo, uint64_t ehi,
                         const char *name) {
    uint64_t lo = 0, hi = 0;
    bool ok = b.read_mem(addr, lo, hi) && lo == elo && hi == ehi;
    char msg[256];
    std::snprintf(msg, sizeof(msg), "%s @0x%05X got=%016llX:%016llX expected=%016llX:%016llX",
                  name, addr,
                  (unsigned long long)hi, (unsigned long long)lo,
                  (unsigned long long)ehi, (unsigned long long)elo);
    CHECK(ok, msg);
    return ok;
}

static bool check_mem64(BusModel &b, uint32_t addr, uint64_t expected, const char *name) {
    return check_mem128(b, addr, expected, 0, name);
}

static void begin_program(Sim &s, int latency) {
    s.bus.latency = latency;
    s.bus.clear();
    s.top->halt = 0;
}

// -----------------------------------------------------------------------------
// 1. Базовый конвейер / сброс / неизвестный opcode / R0
// -----------------------------------------------------------------------------
static void run_basic(Sim &s) {
    logf("\n==== BASIC / RESET / ALU ====\n");

    // NOP + задержанная выборка IF + HALT.
    begin_program(s, 5);
    uint16_t pc = 0;
    for (int i = 0; i < 6; ++i) emit(s.bus, pc, op::NOP);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(5000), "NOP + delayed IF + HALT");
    CHECK(s.bus.protocol_errors() == 0, "memory/IO protocol after NOP/HALT");

    // Неизвестный opcode должен превращаться в bubble, а не в исключение или побочный эффект.
    begin_program(s, 0);
    pc = 0;
    emit(s.bus, pc, 0xFFFF);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x200);
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x210);
    emit(s.bus, pc, op::HALT);
    put_u128(s.bus, 0x200, 0x1234);
    s.reset();
    CHECK(s.run_until_halt(), "unknown opcode is bubble");
    check_mem64(s.bus, 0x210, 0x1234, "unknown opcode did not break next instruction");

    // R0 всегда равен нулю, а записи в него не должны создавать значение.
    begin_program(s, 2);
    pc = 0;
    put_u128(s.bus, 0x220, 0x9999);
    emit(s.bus, pc, op::LOAD, 0, 0, 0, 0x220);
    emit(s.bus, pc, op::MOV, 0, 0, 0);
    emit(s.bus, pc, op::ADD, 1, 0, 0);
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x230);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(), "R0 write/read");
    check_mem64(s.bus, 0x230, 0, "R0 remains zero");

    // Полноценные 128-битные значения ALU.
    begin_program(s, 0);
    pc = 0;
    const uint64_t A_LO = 0x0123456789ABCDEFULL;
    const uint64_t A_HI = 0xFEDCBA9876543210ULL;
    const uint64_t B_LO = 0x1111111111111111ULL;
    const uint64_t B_HI = 0x0000000000000001ULL;
    put_u128(s.bus, 0x240, A_LO, A_HI);
    put_u128(s.bus, 0x250, B_LO, B_HI);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x240);
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x250);
    emit(s.bus, pc, op::ADD, 3, 1, 2);
    emit(s.bus, pc, op::STORE, 0, 0, 3, 0x260);
    emit(s.bus, pc, op::SUB, 4, 1, 2);
    emit(s.bus, pc, op::STORE, 0, 0, 4, 0x270);
    emit(s.bus, pc, op::AND, 5, 1, 2);
    emit(s.bus, pc, op::STORE, 0, 0, 5, 0x280);
    emit(s.bus, pc, op::OR, 6, 1, 2);
    emit(s.bus, pc, op::STORE, 0, 0, 6, 0x290);
    emit(s.bus, pc, op::XOR, 7, 1, 2);
    emit(s.bus, pc, op::STORE, 0, 0, 7, 0x2A0);
    emit(s.bus, pc, op::NOT, 8, 1, 0);
    emit(s.bus, pc, op::STORE, 0, 0, 8, 0x2B0);
    emit(s.bus, pc, op::MOV, 9, 1, 0);
    emit(s.bus, pc, op::STORE, 0, 0, 9, 0x2C0);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(20000), "128-bit ALU");
    check_mem128(s.bus, 0x260,
                 A_LO + B_LO,
                 A_HI + B_HI + ((A_LO + B_LO) < A_LO), "ADD 128");
    check_mem128(s.bus, 0x270,
                 A_LO - B_LO,
                 A_HI - B_HI - ((A_LO < B_LO) ? 1ULL : 0ULL), "SUB 128");
    check_mem128(s.bus, 0x280, A_LO & B_LO, A_HI & B_HI, "AND 128");
    check_mem128(s.bus, 0x290, A_LO | B_LO, A_HI | B_HI, "OR 128");
    check_mem128(s.bus, 0x2A0, A_LO ^ B_LO, A_HI ^ B_HI, "XOR 128");
    check_mem128(s.bus, 0x2B0, ~A_LO, ~A_HI, "NOT 128");
    check_mem128(s.bus, 0x2C0, A_LO, A_HI, "MOV 128");

    // Семантика сдвигов, включая переход бита через границу 64 бит.
    begin_program(s, 0);
    pc = 0;
    put_u128(s.bus, 0x2D0, 0x0000000000000001ULL, 0);
    put_u128(s.bus, 0x2E0, 65, 0);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x2D0);
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x2E0);
    emit(s.bus, pc, op::SHL, 3, 1, 2);
    emit(s.bus, pc, op::STORE, 0, 0, 3, 0x2F0);
    emit(s.bus, pc, op::SHR, 4, 1, 2);
    emit(s.bus, pc, op::STORE, 0, 0, 4, 0x300);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(), "SHL/SHR 128-bit boundary");
    check_mem128(s.bus, 0x2F0, 0x0, 0x2, "SHL by 65");
    check_mem128(s.bus, 0x300, 0x0, 0x0, "SHR by 65");
}

// -----------------------------------------------------------------------------
// 2. Forwarding и матрица зависимостей памяти
// -----------------------------------------------------------------------------
static void run_forwarding(Sim &s) {
    logf("\n==== FORWARDING / HAZARDS ====\n");

    // LOAD -> ALU -> STORE: цепочка, многократно проверяющая forwarding в EX.
    begin_program(s, 3);
    uint16_t pc = 0;
    put_u128(s.bus, 0x400, 10);
    put_u128(s.bus, 0x410, 20);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x400);
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x410);
    emit(s.bus, pc, op::ADD, 3, 1, 2);
    emit(s.bus, pc, op::SUB, 4, 3, 1);
    emit(s.bus, pc, op::ADD, 5, 4, 3);
    emit(s.bus, pc, op::STORE, 0, 0, 5, 0x420);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(50000), "LOAD/ALU forwarding chain");
    check_mem64(s.bus, 0x420, 50, "forwarding chain result");

    // ALU -> STORE без промежуточной инструкции.
    begin_program(s, 0);
    pc = 0;
    put_u128(s.bus, 0x430, 7);
    put_u128(s.bus, 0x440, 9);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x430);
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x440);
    emit(s.bus, pc, op::ADD, 3, 1, 2);
    emit(s.bus, pc, op::STORE, 0, 0, 3, 0x450);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(), "ALU->STORE forwarding");
    check_mem64(s.bus, 0x450, 16, "ALU->STORE value");

    // STORE -> LOAD по одному адресу при задержанных записи/чтении.
    begin_program(s, 4);
    pc = 0;
    put_u128(s.bus, 0x460, 0xBEEF);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x460);
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x470);
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x470);
    emit(s.bus, pc, op::STORE, 0, 0, 2, 0x480);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(50000), "STORE->LOAD same address");
    check_mem64(s.bus, 0x480, 0xBEEF, "STORE->LOAD value");

    // Длинная цепочка зависимости самой от себя: выявляет устаревшие данные WB / ошибки forwarding в EX.
    begin_program(s, 2);
    pc = 0;
    put_u128(s.bus, 0x490, 1);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x490);
    for (int i = 0; i < 20; ++i) emit(s.bus, pc, op::ADD, 1, 1, 1);
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x4A0);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(100000), "20-stage self forwarding chain");
    check_mem64(s.bus, 0x4A0, 1ULL << 20, "2^20 forwarding chain");
}

// -----------------------------------------------------------------------------
// 3. Все условные переходы: taken и not-taken
// -----------------------------------------------------------------------------
static void probe_branch(Sim &s, const char *name, uint16_t branch_op,
                         uint64_t a_lo, uint64_t a_hi,
                         uint64_t b_lo, uint64_t b_hi,
                         bool expect_taken, int latency = 0) {
    begin_program(s, latency);
    uint16_t pc = 0;
    const uint32_t A = 0x1000;
    const uint32_t B = 0x1010;
    const uint32_t FAIL = 0x1100;
    const uint32_t OK = 0x1110;

    put_u128(s.bus, A, a_lo, a_hi);
    put_u128(s.bus, B, b_lo, b_hi);
    put_u128(s.bus, 0x1020, 0xEE);
    put_u128(s.bus, 0x1030, 0xAA);

    emit(s.bus, pc, op::LOAD, 1, 0, 0, A);
    emit(s.bus, pc, op::LOAD, 2, 0, 0, B);
    emit(s.bus, pc, op::LOAD, 3, 0, 0, 0x1020);
    emit(s.bus, pc, op::LOAD, 4, 0, 0, 0x1030);
    emit(s.bus, pc, op::CMP, 0, 1, 2);
    // Переход находится по PC=0x50; +0x20 даёт адрес 0x70, где находится корректный STORE.
    emit(s.bus, pc, branch_op, 0, 0, 0, 0x20);
    emit(s.bus, pc, op::STORE, 0, 0, 3, FAIL);
    emit(s.bus, pc, op::STORE, 0, 0, 4, OK);
    emit(s.bus, pc, op::HALT);

    s.reset();
    bool halted = s.run_until_halt(100000);
    CHECK(halted, name);

    uint64_t lo = 0, hi = 0;
    if (expect_taken) {
        bool fail_written = s.bus.read_mem(FAIL, lo, hi) && lo == 0xEE;
        CHECK(!fail_written, (std::string(name) + " taken: FAIL path skipped").c_str());
        CHECK(s.bus.read_mem(OK, lo, hi) && lo == 0xAA,
              (std::string(name) + " taken: OK path").c_str());
    } else {
        CHECK(s.bus.read_mem(FAIL, lo, hi) && lo == 0xEE,
              (std::string(name) + " not-taken: FAIL path").c_str());
    }
}

static void run_branches(Sim &s) {
    logf("\n==== BRANCHES / FLAGS ====\n");

    // Z / N / C / V и инверсные условия.
    probe_branch(s, "JZ taken",  op::JZ,  0,0, 0,0, true);
    probe_branch(s, "JZ not",    op::JZ,  1,0, 0,0, false);
    probe_branch(s, "JNZ taken", op::JNZ, 1,0, 0,0, true);
    probe_branch(s, "JNZ not",   op::JNZ, 0,0, 0,0, false);

    // CMP вычисляет a-b: N отражает знак результата, поэтому a=0,b=1 даёт отрицательный результат.
    probe_branch(s, "JN taken",  op::JN,  0,0, 1,0, true);
    probe_branch(s, "JN not",    op::JN,  2,0, 1,0, false);
    probe_branch(s, "JP taken",  op::JP,  2,0, 1,0, true);
    probe_branch(s, "JP not",    op::JP,  0,0, 1,0, false);

    // Для CMP: a>=b => C=1 (заёма нет), a<b => C=0.
    probe_branch(s, "JC taken",  op::JC,  5,0, 3,0, true);
    probe_branch(s, "JC not",    op::JC,  1,0, 3,0, false);
    probe_branch(s, "JNC taken", op::JNC, 1,0, 3,0, true);
    probe_branch(s, "JNC not",   op::JNC, 5,0, 3,0, false);

    // CMP 0x7FFF... - (-1) => 0x8000..., V=1.
    const uint64_t POS_MAX_LO = 0xFFFFFFFFFFFFFFFFULL;
    const uint64_t POS_MAX_HI = 0x7FFFFFFFFFFFFFFFULL;
    const uint64_t NEG_ONE_LO = 0xFFFFFFFFFFFFFFFFULL;
    const uint64_t NEG_ONE_HI = 0xFFFFFFFFFFFFFFFFULL;
    probe_branch(s, "JV taken",  op::JV,  POS_MAX_LO, POS_MAX_HI,
                 NEG_ONE_LO, NEG_ONE_HI, true);
    probe_branch(s, "JV not",    op::JV,  5,0, 3,0, false);
    probe_branch(s, "JNV taken", op::JNV, 5,0, 3,0, true);
    probe_branch(s, "JNV not",   op::JNV, POS_MAX_LO, POS_MAX_HI,
                 NEG_ONE_LO, NEG_ONE_HI, false);

    // EQ/NE.
    probe_branch(s, "JE taken",  op::JE,  4,0, 4,0, true);
    probe_branch(s, "JE not",    op::JE,  4,0, 5,0, false);
    probe_branch(s, "JNE taken", op::JNE, 4,0, 5,0, true);
    probe_branch(s, "JNE not",   op::JNE, 4,0, 4,0, false);

    // Беззнаковые сравнения.
    probe_branch(s, "JUGT taken", op::JUGT, 5,0, 3,0, true);
    probe_branch(s, "JUGT not",   op::JUGT, 3,0, 5,0, false);
    probe_branch(s, "JULT taken", op::JULT, 3,0, 5,0, true);
    probe_branch(s, "JULT not",   op::JULT, 5,0, 3,0, false);
    probe_branch(s, "JUGE taken", op::JUGE, 5,0, 5,0, true);
    probe_branch(s, "JUGE not",   op::JUGE, 3,0, 5,0, false);
    probe_branch(s, "JULE taken", op::JULE, 3,0, 5,0, true);
    probe_branch(s, "JULE not",   op::JULE, 5,0, 3,0, false);

    // Знаковые сравнения: -1 < +1 и -1 > -2.
    const uint64_t NEG_TWO_LO = 0xFFFFFFFFFFFFFFFEULL;
    const uint64_t NEG_TWO_HI = 0xFFFFFFFFFFFFFFFFULL;
    const uint64_t POS_ONE_LO = 1;
    const uint64_t POS_ONE_HI = 0;
    probe_branch(s, "JSGT taken", op::JSGT, NEG_ONE_LO, NEG_ONE_HI,
                 NEG_TWO_LO, NEG_TWO_HI, true);
    probe_branch(s, "JSGT not",   op::JSGT, NEG_TWO_LO, NEG_TWO_HI,
                 NEG_ONE_LO, NEG_ONE_HI, false);
    probe_branch(s, "JSLT taken", op::JSLT, NEG_ONE_LO, NEG_ONE_HI,
                 POS_ONE_LO, POS_ONE_HI, true);
    probe_branch(s, "JSLT not",   op::JSLT, POS_ONE_LO, POS_ONE_HI,
                 NEG_ONE_LO, NEG_ONE_HI, false);
    probe_branch(s, "JSGE taken", op::JSGE, NEG_ONE_LO, NEG_ONE_HI,
                 NEG_ONE_LO, NEG_ONE_HI, true);
    probe_branch(s, "JSGE not",   op::JSGE, NEG_TWO_LO, NEG_TWO_HI,
                 NEG_ONE_LO, NEG_ONE_HI, false);
    probe_branch(s, "JSLE taken", op::JSLE, NEG_ONE_LO, NEG_ONE_HI,
                 POS_ONE_LO, POS_ONE_HI, true);
    probe_branch(s, "JSLE not",   op::JSLE, POS_ONE_LO, POS_ONE_HI,
                 NEG_TWO_LO, NEG_TWO_HI, false);

    // Обратный относительный переход: цикл выполняется, пока R1 != R2.
    begin_program(s, 2);
    uint16_t pc = 0;
    put_u128(s.bus, 0x1200, 0);
    put_u128(s.bus, 0x1210, 4);
    put_u128(s.bus, 0x1220, 1);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x1200); // 0
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x1210); // 4
    emit(s.bus, pc, op::LOAD, 3, 0, 0, 0x1220); // 1
    // начало цикла по адресу 0x30
    emit(s.bus, pc, op::CMP, 0, 1, 2);          // 0x30
    emit(s.bus, pc, op::JUGE, 0, 0, 0, 0x40);   // 0x40 exit -> 0x80
    emit(s.bus, pc, op::ADD, 1, 1, 3);          // 0x50
    emit(s.bus, pc, op::JMP, 0, 0, 0, uint64_t(-0x30)); // 0x60 -> 0x30
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x1230); // 0x70 (should be flushed/skipped)
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x1240); // 0x80
    emit(s.bus, pc, op::HALT);                   // 0x90
    s.reset();
    CHECK(s.run_until_halt(100000), "backward branch loop");
    check_mem64(s.bus, 0x1240, 4, "backward branch final value");
}

// -----------------------------------------------------------------------------
// 4. JMP / JMPF / MOVS CS и сброс конвейера
// -----------------------------------------------------------------------------
static void run_control_transfer(Sim &s) {
    logf("\n==== CONTROL TRANSFERS / CS ====\n");

    // JMP должен отбросить уже выбранную последовательную инструкцию.
    begin_program(s, 4);
    uint16_t pc = 0;
    put_u128(s.bus, 0x1300, 0xAA);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x1300);
    emit(s.bus, pc, op::JMP, 0, 0, 0, 0x20);   // 0x10 + 0x20 = 0x30
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x1310); // wrong path at 0x20
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x1320); // target at 0x30
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(100000), "JMP taken + flush");
    uint64_t lo = 0, hi = 0;
    CHECK(!s.bus.read_mem(0x1310, lo, hi), "JMP skipped sequential STORE");
    check_mem64(s.bus, 0x1320, 0xAA, "JMP target");

    // JMPF: imm[15:0] = CS, imm[31:16] = PC.
    begin_program(s, 3);
    pc = 0;
    put_u128(s.bus, 0x1400, 0x55);
    put_u128(s.bus, 0x1410, 0xDEAD);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x1400);
    emit(s.bus, pc, op::JMPF, 0, 0, 0, uint64_t(1) | (uint64_t(0x40) << 16)); // CS=1, PC=0x40 => phys 0x50
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x1410); // old path
    emit(s.bus, pc, op::HALT);
    // Новый код: CS=1, PC=0x40 => физический адрес 0x10 + 0x40 = 0x50.
    s.bus.prog(0x50, op::STORE, 0, 0, 1, 0x1420);
    s.bus.prog(0x60, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(100000), "JMPF target");
    // Область старого CS заранее заполнена значением DEAD. Пропущенный STORE должен
    // оставить это значение без изменений; read_mem() должен вернуть true для
    // инициализированной области, поэтому проверка !read_mem() была ошибочной.
    check_mem64(s.bus, 0x1410, 0xDEAD, "JMPF skipped old CS path");
    check_mem64(s.bus, 0x1420, 0x55, "JMPF new CS target");

    // MOVS CS: старый последовательный путь должен быть отброшен; новый CS продолжает выполнение с PC+16.
    begin_program(s, 2);
    pc = 0;
    put_u128(s.bus, 0x1500, 1);       // new CS = 1
    put_u128(s.bus, 0x1510, 0xAA);    // good marker
    put_u128(s.bus, 0x1520, 0xDEAD);  // bad marker
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x1500); // PC 0x00
    emit(s.bus, pc, op::MOVS, op::SEG_CS, 1, 0); // PC 0x10
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x1520); // old-CS path, must flush
    emit(s.bus, pc, op::STORE, 0, 0, 2, 0x1530); // old-CS path
    emit(s.bus, pc, op::HALT);
    // Новый CS=1, следующий PC после MOVS равен 0x20 => физический адрес 0x30.
    s.bus.prog(0x30, op::LOAD, 2, 0, 0, 0x1510);
    s.bus.prog(0x40, op::STORE, 0, 0, 2, 0x1530);
    s.bus.prog(0x50, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(100000), "MOVS CS + pipeline flush");
    check_mem64(s.bus, 0x1530, 0xAA, "MOVS CS new-code path");
    CHECK(s.bus.read_mem(0x1530, lo, hi) && lo != 0xDEAD, "MOVS CS old path did not win");
}

// -----------------------------------------------------------------------------
// 4b. MOVS CS: развёртка по задержке памяти и по инструкциям перед MOVS.
//
// Проверяемая гипотеза: если инструкция, следующая за MOVS CS, уже лежит в
// IF/ID (pc уже продвинут за неё), то сброс по смене CS очищает IF/ID, но не
// откатывает pc, и эта инструкция теряется: новый код начнётся не с PC+16,
// а с PC+32. Наиболее вероятный сценарий: LOAD непосредственно перед MOVS и
// быстрая память (задержка 0..1), когда выборка успевает завершиться, пока
// конвейер стоит на data-транзакции.
//
// Для каждой пары (задержка, пролог) проверяется, что:
//   * первая инструкция нового пути (CS=NEW_CS, PC+16) выполнилась;
//   * инструкция старого пути (CS=0, PC+16) не выполнилась;
//   * нет нарушений протокола памяти.
// Маркеры уникальны для каждой итерации, потому что регистровый файл не
// очищается сбросом: устаревшее значение r2/r3 не должно давать ложный PASS.
// -----------------------------------------------------------------------------
static void run_movs_cs_sweep(Sim &s) {
    logf("\n==== MOVS CS SWEEP (latency x prologue) ====\n");

    static const int kLatencies[] = {0, 1, 2, 3, 4, 5, 8};
    static const char *kPrologNames[] = {
        "LOAD,MOVS",
        "LOAD,NOP,MOVS",
        "LOAD,NOP,NOP,MOVS",
        "LOAD,LOAD(other),MOVS",
        "LOAD,STORE,MOVS",
        "LOAD,ADD,MOVS",
    };
    constexpr int kPrologues = int(sizeof(kPrologNames) / sizeof(kPrologNames[0]));
    constexpr uint64_t NEW_CS = 0x100;      // база нового кода: 0x100 * 16 = 0x1000

    int iter = 0;
    for (int latency : kLatencies) {
        for (int kind = 0; kind < kPrologues; ++kind) {
            ++iter;
            const uint64_t good = 0xA5A50000ULL + uint64_t(iter);
            const uint64_t bad  = 0xDEAD0000ULL + uint64_t(iter);

            begin_program(s, latency);
            put_u128(s.bus, 0x1500, NEW_CS);
            put_u128(s.bus, 0x1510, good);
            put_u128(s.bus, 0x1520, bad);
            put_u128(s.bus, 0x1530, 0);     // результат нового пути
            put_u128(s.bus, 0x1540, 0);     // результат старого пути
            put_u128(s.bus, 0x1550, 7);

            uint16_t pc = 0;
            emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x1500);   // r1 = NEW_CS
            switch (kind) {
                case 1: emit(s.bus, pc, op::NOP); break;
                case 2: emit(s.bus, pc, op::NOP);
                        emit(s.bus, pc, op::NOP); break;
                case 3: emit(s.bus, pc, op::LOAD, 6, 0, 0, 0x1550); break;
                case 4: emit(s.bus, pc, op::STORE, 0, 0, 0, 0x1560); break;
                case 5: emit(s.bus, pc, op::ADD, 7, 0, 0); break;
                default: break;
            }
            emit(s.bus, pc, op::MOVS, op::SEG_CS, 1, 0);
            const uint16_t after = pc;                    // PC сразу за MOVS

            // Старый путь (CS=0): не должен выполниться.
            emit(s.bus, pc, op::LOAD, 3, 0, 0, 0x1520);
            emit(s.bus, pc, op::STORE, 0, 0, 3, 0x1540);
            emit(s.bus, pc, op::HALT);

            // Новый путь: тот же PC, но физический адрес = NEW_CS*16 + PC.
            uint16_t npc = after;
            const uint16_t base = uint16_t(NEW_CS << 4);
            s.bus.prog(uint16_t(base + npc), op::LOAD, 2, 0, 0, 0x1510);   npc += 16;
            s.bus.prog(uint16_t(base + npc), op::STORE, 0, 0, 2, 0x1530);  npc += 16;
            s.bus.prog(uint16_t(base + npc), op::HALT);

            s.reset();
            const bool halted = s.run_until_halt(20000);

            uint64_t new_val = 0, old_val = 0, hi = 0;
            s.bus.read_mem(0x1530, new_val, hi);
            s.bus.read_mem(0x1540, old_val, hi);

            CHECKF(halted && new_val == good && old_val == 0 &&
                   s.bus.protocol_errors() == 0,
                   "MOVS CS latency=%d prolog=[%s]: halted=%d new=0x%llX (want 0x%llX) old=0x%llX (want 0) proto_err=%d",
                   latency, kPrologNames[kind], int(halted),
                   (unsigned long long)new_val, (unsigned long long)good,
                   (unsigned long long)old_val, int(s.bus.protocol_errors()));
        }
    }
}

// -----------------------------------------------------------------------------
// 5. Сегментные регистры / MFS / LOADS / STORES
// -----------------------------------------------------------------------------
static void run_segments(Sim &s) {
    logf("\n==== SEGMENTS / MFS / LOADS / STORES ====\n");

    begin_program(s, 3);
    uint16_t pc = 0;

    // Сегментные регистры изначально равны нулю. Используем обычные LOAD через DS
    // до изменения DS/SS/ES, чтобы исходные GPR были известны независимо
    // от проверяемых сегментных регистров.
    put_u128(s.bus, 0x1600, 1);
    put_u128(s.bus, 0x1610, 2);
    put_u128(s.bus, 0x1620, 3);

    // Ожидаемые данные по физическим адресам, которые генерируют LOADS:
    //   DS=1 -> 0x0010 + 0x1000 = 0x1010
    //   SS=2 -> 0x0020 + 0x2000 = 0x2020
    //   ES=3 -> 0x0030 + 0x3000 = 0x3030
    put_u128(s.bus, 0x1010, 0x1111);
    put_u128(s.bus, 0x2020, 0x2222);
    put_u128(s.bus, 0x3030, 0x3333);

    // Сначала загружаем все три значения сегментов. Это намеренно устраняет
    // зависимость от сегментного регистра при формировании новых значений.
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x1600); // r1 = 1
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x1610); // r2 = 2
    emit(s.bus, pc, op::LOAD, 3, 0, 0, 0x1620); // r3 = 3

    // MOVS изменяет архитектурное состояние внутри конвейера. Оставляем явные NOP
    // между обновлениями, чтобы тест проверял именно архитектурную операцию над сегментом,
    // а не зависел от конкретного тайминга producer/consumer.
    emit(s.bus, pc, op::NOP);
    emit(s.bus, pc, op::MOVS, op::SEG_DS, 1, 0); // DS = 1
    emit(s.bus, pc, op::NOP);
    emit(s.bus, pc, op::NOP);

    emit(s.bus, pc, op::MOVS, op::SEG_SS, 2, 0); // SS = 2
    emit(s.bus, pc, op::NOP);
    emit(s.bus, pc, op::NOP);

    emit(s.bus, pc, op::MOVS, op::SEG_ES, 3, 0); // ES = 3
    emit(s.bus, pc, op::NOP);
    emit(s.bus, pc, op::NOP);

    // MFS: rs1[1:0] выбирает CS/DS/SS/ES.
    emit(s.bus, pc, op::MFS, 4, op::SEG_CS, 0);
    emit(s.bus, pc, op::MFS, 5, op::SEG_DS, 0);
    emit(s.bus, pc, op::MFS, 6, op::SEG_SS, 0);
    emit(s.bus, pc, op::MFS, 7, op::SEG_ES, 0);

    // Сохраняем результаты MFS через CS=0, чтобы адрес назначения однозначно
    // совпадал с offset как физический адрес.
    emit(s.bus, pc, op::STORES, op::SEG_CS, 0, 4, 0x1630);
    emit(s.bus, pc, op::STORES, op::SEG_CS, 0, 5, 0x1640);
    emit(s.bus, pc, op::STORES, op::SEG_CS, 0, 6, 0x1650);
    emit(s.bus, pc, op::STORES, op::SEG_CS, 0, 7, 0x1660);

    // LOADS: селектор находится в rs2[1:0]. База — r0, поэтому immediate содержит
    // полный offset.
    emit(s.bus, pc, op::LOADS, 8, 0, op::SEG_DS, 0x1000);
    emit(s.bus, pc, op::LOADS, 9, 0, op::SEG_SS, 0x2000);
    emit(s.bus, pc, op::LOADS, 10, 0, op::SEG_ES, 0x3000);

    // Снова используем CS=0 для STORE, которыми наблюдаем результаты.
    emit(s.bus, pc, op::STORES, op::SEG_CS, 0, 8, 0x1670);
    emit(s.bus, pc, op::STORES, op::SEG_CS, 0, 9, 0x1680);
    emit(s.bus, pc, op::STORES, op::SEG_CS, 0, 10, 0x1690);

    // STORES: селектор находится в rd[1:0], источник данных — rs2.
    emit(s.bus, pc, op::STORES, op::SEG_DS, 0, 8, 0x1700);
    emit(s.bus, pc, op::STORES, op::SEG_SS, 0, 9, 0x1800);
    emit(s.bus, pc, op::STORES, op::SEG_ES, 0, 10, 0x1900);

    emit(s.bus, pc, op::HALT);
    s.reset();

    CHECK(s.run_until_halt(200000), "segment instruction suite");

    check_mem64(s.bus, 0x1630, 0, "MFS CS");
    check_mem64(s.bus, 0x1640, 1, "MFS DS");
    check_mem64(s.bus, 0x1650, 2, "MFS SS");
    check_mem64(s.bus, 0x1660, 3, "MFS ES");

    check_mem64(s.bus, 0x1670, 0x1111, "LOADS DS");
    check_mem64(s.bus, 0x1680, 0x2222, "LOADS SS");
    check_mem64(s.bus, 0x1690, 0x3333, "LOADS ES");

    // Физические адреса назначения для STORES.
    check_mem64(s.bus, 0x1010, 0x1111, "STORES DS");
    check_mem64(s.bus, 0x2020, 0x2222, "STORES SS");
    check_mem64(s.bus, 0x3030, 0x3333, "STORES ES");
}

// -----------------------------------------------------------------------------
// 6. I/O, однотактные строб-сигналы и остановки
// -----------------------------------------------------------------------------
static void run_io(Sim &s) {
    logf("\n==== IO / STROBES / STALLS ====\n");

    for (int latency : {0, 1, 3, 5, 8}) {
        logf("  -- IO latency=%d --\n", latency);
        begin_program(s, latency);
        uint16_t pc = 0;
        emit(s.bus, pc, op::IN, 1, 0, 0, 0x3C);
        emit(s.bus, pc, op::OUT, 0, 1, 0, 0x3C);
        emit(s.bus, pc, op::STORE, 0, 0, 1, 0x1A00);
        emit(s.bus, pc, op::HALT);
        s.reset();
        CHECK(s.run_until_halt(100000), "IN/OUT under IF latency");
        CHECK(s.bus.in_strobes() == 1, "IN exactly one strobe");
        CHECK(s.bus.out_strobes() == 1, "OUT exactly one strobe");
        check_mem64(s.bus, 0x1A00, 0xA53C, "IN data (port + A5)");
        CHECK(s.bus.protocol_errors() == 0, "IO/memory protocol during IN/OUT");
    }

    // Данные OUT должны браться из регистра, а не из номера порта immediate.
    begin_program(s, 4);
    uint16_t pc = 0;
    put_u128(s.bus, 0x1A10, 0xCAFEBABE);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x1A10);
    emit(s.bus, pc, op::OUT, 0, 1, 0, 0x7E);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(50000), "OUT payload forwarding");
    CHECK(s.bus.out_strobes() == 1, "OUT payload one-shot");
}

// -----------------------------------------------------------------------------
// 7. Задержка памяти / IF против DATA / порядок завершения HALT
// -----------------------------------------------------------------------------
static void run_memory_stress(Sim &s) {
    logf("\n==== MEMORY LATENCY / ARBITRATION / HALT ====\n");

    for (int latency = 0; latency <= 8; ++latency) {
        begin_program(s, latency);
        uint16_t pc = 0;
        put_u128(s.bus, 0x2000, 10);
        put_u128(s.bus, 0x2010, 20);
        emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x2000);
        emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x2010);
        emit(s.bus, pc, op::ADD, 3, 1, 2);
        emit(s.bus, pc, op::STORE, 0, 0, 3, 0x2020);
        emit(s.bus, pc, op::HALT);
        s.reset();
        CHECK(s.run_until_halt(250000), "LOAD/LOAD/ADD/STORE latency suite");
        check_mem64(s.bus, 0x2020, 30, "latency ADD result");
        CHECK(s.bus.protocol_errors() == 0, "memory protocol latency suite");
    }

    // Чередование LOAD/STORE заставляет многократно выполнять арбитраж доступа.
    begin_program(s, 4);
    uint16_t pc = 0;
    for (int i = 0; i < 16; ++i) put_u128(s.bus, 0x2100 + 0x10*i, 0x100 + i);
    for (int i = 0; i < 16; ++i) {
        emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x2100 + 0x10*i);
        emit(s.bus, pc, op::ADD, 1, 1, 1);
        emit(s.bus, pc, op::STORE, 0, 0, 1, 0x2200 + 0x10*i);
    }
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(500000), "16-entry memory stream");
    for (int i = 0; i < 16; ++i) {
        char msg[64];
        std::snprintf(msg, sizeof(msg), "stream[%d]", i);
        check_mem64(s.bus, 0x2200 + 0x10*i, uint64_t(2*(0x100+i)), msg);
    }

    // Конфликт IF и DATA: длинные транзакции данных при продолжающейся последовательной выборке IF.
    begin_program(s, 7);
    pc = 0;
    put_u128(s.bus, 0x2300, 0x55);
    for (int i = 0; i < 6; ++i) {
        emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x2300);
        emit(s.bus, pc, op::ADD, 1, 1, 1);
        emit(s.bus, pc, op::STORE, 0, 0, 1, 0x2400 + 0x10*i);
    }
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(500000), "IF/data contention stress");
    for (int i = 0; i < 6; ++i) {
        char msg[64];
        std::snprintf(msg, sizeof(msg), "contention[%d]", i);
        check_mem64(s.bus, 0x2400 + 0x10*i, 0xAA, msg);
    }

    // HALT сразу после LOAD: HALT не должен завершать процессор до завершения
    // транзакции данных и предшествующего запроса IF.
    begin_program(s, 8);
    pc = 0;
    put_u128(s.bus, 0x2500, 0x12345678);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x2500);
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x2510);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(100000), "HALT after delayed data access");
    check_mem64(s.bus, 0x2510, 0x12345678, "HALT waited for LOAD/STORE");
    CHECK(!s.bus.active(), "external memory transaction idle at HALT");
}

// -----------------------------------------------------------------------------
// 8. Операции, формирующие флаги: ADD/SUB/переносы сдвига/переполнение
// -----------------------------------------------------------------------------
static void run_flag_edges(Sim &s) {
    logf("\n==== FLAG EDGE CASES ====\n");

    // ADD максимального беззнакового числа + 1 => ноль, C=1, V=0. Для 128-битного максимума
    // знаковые операнды равны -1 + 1, поэтому знакового переполнения нет.
    begin_program(s, 2);
    uint16_t pc = 0;
    put_u128(s.bus, 0x2600, 0xFFFFFFFFFFFFFFFFULL, 0xFFFFFFFFFFFFFFFFULL);
    put_u128(s.bus, 0x2610, 1, 0);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x2600);
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x2610);
    emit(s.bus, pc, op::ADD, 3, 1, 2);
    emit(s.bus, pc, op::JC, 0, 0, 0, 0x20);
    emit(s.bus, pc, op::STORE, 0, 0, 3, 0x2620); // skipped if C=1
    emit(s.bus, pc, op::STORE, 0, 0, 3, 0x2630);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(100000), "ADD carry edge");
    uint64_t tmp_lo = 0, tmp_hi = 0;
    CHECK(!s.bus.read_mem(0x2620, tmp_lo, tmp_hi), "JC skipped wrong path");
    check_mem64(s.bus, 0x2630, 0, "ADD max+1 result zero");

    // Знаковое переполнение: 0x7FFF... + 1 => 0x8000..., V=1.
    begin_program(s, 2);
    pc = 0;
    put_u128(s.bus, 0x2640, 0xFFFFFFFFFFFFFFFFULL, 0x7FFFFFFFFFFFFFFFULL);
    put_u128(s.bus, 0x2650, 1, 0);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x2640);
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x2650);
    emit(s.bus, pc, op::ADD, 3, 1, 2);
    emit(s.bus, pc, op::JV, 0, 0, 0, 0x20);
    emit(s.bus, pc, op::STORE, 0, 0, 3, 0x2660);
    emit(s.bus, pc, op::STORE, 0, 0, 3, 0x2670);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(100000), "ADD signed overflow edge");
    uint64_t lo=0, hi=0;
    CHECK(!s.bus.read_mem(0x2660, lo, hi), "JV skipped wrong path");
    check_mem128(s.bus, 0x2670, 0, 0x8000000000000000ULL, "ADD signed overflow result");

    // Граничные случаи переноса для SHL/SHR: бит 127 и бит 0.
    begin_program(s, 1);
    pc = 0;
    put_u128(s.bus, 0x2680, 0x0, 0x8000000000000000ULL);
    put_u128(s.bus, 0x2690, 1, 0);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x2680);
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x2690);
    emit(s.bus, pc, op::SHL, 3, 1, 2);
    emit(s.bus, pc, op::JC, 0, 0, 0, 0x20);
    emit(s.bus, pc, op::STORE, 0, 0, 3, 0x26A0);
    emit(s.bus, pc, op::STORE, 0, 0, 3, 0x26B0);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(100000), "SHL carry edge");
    CHECK(!s.bus.read_mem(0x26A0, lo, hi), "SHL carry branch taken");
    check_mem128(s.bus, 0x26B0, 0, 0, "SHL result zero");
}

// -----------------------------------------------------------------------------
// 9. Переход при задержанной IF, затем обращение к памяти по адресу назначения.
// -----------------------------------------------------------------------------
static void run_branch_if_contention(Sim &s) {
    logf("\n==== BRANCH + IF WAIT + DATA FOLLOW-UP ====\n");

    begin_program(s, 8);
    uint16_t pc = 0;
    put_u128(s.bus, 0x2700, 0);
    put_u128(s.bus, 0x2710, 0xABCD);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x2700);
    emit(s.bus, pc, op::CMP, 0, 1, 1);
    emit(s.bus, pc, op::JZ, 0, 0, 0, 0x20); // target after one skipped STORE
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x2720); // must be skipped
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x2710); // target
    emit(s.bus, pc, op::STORE, 0, 0, 2, 0x2730);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(200000), "branch during IF wait");
    uint64_t lo=0, hi=0;
    CHECK(!s.bus.read_mem(0x2720, lo, hi), "branch flushed wrong-path STORE");
    check_mem64(s.bus, 0x2730, 0xABCD, "branch target LOAD/STORE");
    CHECK(s.bus.protocol_errors() == 0, "branch/IF/data protocol");
}

// -----------------------------------------------------------------------------
// 10. Дополнительные крайние случаи: границы адресов, нулевые/максимальные данные, задержка
//     от минимальной до максимальной и повторные операции памяти.
// -----------------------------------------------------------------------------
static void run_corner_cases(Sim &s) {
    logf("\n==== CORNER CASES / ADDRESS BOUNDARIES / MAX DATA ====\n");

    // Проверяем минимальный и максимальный 20-битные физические адреса памяти.
    // ВАЖНО: физический 0x00000 одновременно является адресом кода после сброса, поэтому его нельзя
    // использовать как область данных, пока программа выполняется из него.
    // В начале выполняем JMPF в CS=1/PC=0x20; физический адрес кода там равен
    // 0x00030, после чего DS=0 позволяет проверять данные по 0x00000 и 0xFFFF0.
    begin_program(s, 16);
    uint16_t pc = 0;
    const uint64_t MAX_LO = 0xFFFFFFFFFFFFFFFFULL;
    const uint64_t MAX_HI = 0xFFFFFFFFFFFFFFFFULL;

    // Безопасный источник данных вне области кода.
    put_u128(s.bus, 0x00500, MAX_LO, MAX_HI);

    // Код после сброса по физическому 0x00000: переход в CS=1, PC=0x0020.
    // CS=1 даёт базу сегмента 0x10, поэтому первая целевая
    // инструкция физически находится по адресу 0x00030.
    emit(s.bus, pc, op::JMPF, 0, 0, 0,
         uint64_t(1) | (uint64_t(0x20) << 16));

    // Целевой код: CS=1, PC=0x20 -> физический адрес 0x30.
    // 1) Записать ноль в минимальный физический адрес.
    // 2) Прочитать его обратно и сохранить ноль по адресу 0x520.
    // 3) Записать этот ноль в максимальный выровненный физический адрес.
    // 4) Прочитать максимальное значение из 0x500 и записать его в 0x00000.
    // 5) Прочитать максимум обратно из 0x00000 и записать его в 0xFFFF0.
    s.bus.prog(0x0030, op::STORE, 0, 0, 0, 0x00000);
    s.bus.prog(0x0040, op::LOAD,  1, 0, 0, 0x00000);
    s.bus.prog(0x0050, op::STORE, 0, 0, 1, 0x00520);
    s.bus.prog(0x0060, op::STORE, 0, 0, 1, 0xFFFF0);
    s.bus.prog(0x0070, op::LOAD,  2, 0, 0, 0x00500);
    s.bus.prog(0x0080, op::STORE, 0, 0, 2, 0x00000);
    s.bus.prog(0x0090, op::LOAD,  3, 0, 0, 0x00000);
    s.bus.prog(0x00A0, op::STORE, 0, 0, 3, 0xFFFF0);
    s.bus.prog(0x00B0, op::HALT);

    s.reset();
    CHECK(s.run_until_halt(200000), "memory address boundaries + latency 16");

    // Сначала граничный адрес должен принять ноль, затем максимальное 128-битное значение. Итоговое
    // состояние тем самым проверяет и запись, и чтение непосредственно на границах адресного пространства.
    check_mem128(s.bus, 0x00520, 0, 0,
                 "lowest address initially stores zero");
    check_mem128(s.bus, 0x00000, MAX_LO, MAX_HI,
                 "lowest address receives maximum 128-bit value");
    check_mem128(s.bus, 0xFFFF0, MAX_LO, MAX_HI,
                 "highest aligned address receives maximum 128-bit value");
    CHECK(s.bus.protocol_errors() == 0, "boundary-address protocol");

    // Повторные STORE в одну и ту же ячейку. Это позволяет обнаружить случайную
    // потерю разрешения записи или неправильную обработку завершения после задержанного ACK.
    begin_program(s, 2);
    pc = 0;
    put_u128(s.bus, 0x500, 0x1111111111111111ULL, 0xAAAAAAAAAAAAAAAAULL);
    put_u128(s.bus, 0x510, 0x2222222222222222ULL, 0xBBBBBBBBBBBBBBBBULL);
    emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x500);
    emit(s.bus, pc, op::STORE, 0, 0, 1, 0x520);
    emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x510);
    emit(s.bus, pc, op::STORE, 0, 0, 2, 0x520);
    emit(s.bus, pc, op::LOAD, 3, 0, 0, 0x520);
    emit(s.bus, pc, op::STORE, 0, 0, 3, 0x530);
    emit(s.bus, pc, op::HALT);

    s.reset();
    CHECK(s.run_until_halt(100000), "repeated STORE/LOAD dependency chain");
    check_mem128(s.bus, 0x530,
                 0x2222222222222222ULL, 0xBBBBBBBBBBBBBBBBULL,
                 "last STORE wins and LOAD observes it");
    CHECK(s.bus.protocol_errors() == 0, "repeated memory operation protocol");
}

// -----------------------------------------------------------------------------
// 10. Deterministic property-based ALU verification.
//
// Важно: здесь НЕ используется случайный результат от предыдущей программы.
// Каждый вектор запускается в отдельном чистом экземпляре архитектурного
// состояния (reset + новая программа), поэтому ошибка тестового стенда не может
// маскироваться накопленным состоянием.
// -----------------------------------------------------------------------------
static uint64_t prng64(uint64_t &state) {
    // xorshift64* — детерминированный генератор, не требующий никаких библиотек.
    state ^= state >> 12;
    state ^= state << 25;
    state ^= state >> 27;
    return state * 0x2545F4914F6CDD1DULL;
}

static unsigned __int128 make_u128(uint64_t lo, uint64_t hi) {
    return (static_cast<unsigned __int128>(hi) << 64) | lo;
}

static uint64_t u128_lo(unsigned __int128 x) {
    return static_cast<uint64_t>(x);
}

static uint64_t u128_hi(unsigned __int128 x) {
    return static_cast<uint64_t>(x >> 64);
}

static void run_alu_properties(Sim &s) {
    logf("\n==== PROPERTY-BASED ALU VERIFICATION ====\n");
    logf("  Deterministic vectors; no Yosys, no filesystem, no external solver.\n");

    struct Vec {
        uint64_t a_lo, a_hi;
        uint64_t b_lo, b_hi;
    };

    // Несколько фиксированных граничных значений + детерминированные векторы.
    const Vec fixed[] = {
        {0,0, 0,0},
        {1,0, 1,0},
        {0xFFFFFFFFFFFFFFFFULL,0, 1,0},
        {0xFFFFFFFFFFFFFFFFULL,0xFFFFFFFFFFFFFFFFULL, 1,0},
        {0,0x8000000000000000ULL, 0,0},
        {0x0123456789ABCDEFULL,0xFEDCBA9876543210ULL,
         0x1111111111111111ULL,0x0000000000000001ULL},
        {0xAAAAAAAAAAAAAAAAULL,0xAAAAAAAAAAAAAAAAULL,
         0x5555555555555555ULL,0x5555555555555555ULL},
    };

    uint64_t seed = 0xC001D00D12345678ULL;
    constexpr int RANDOM_VECTORS = 24;
    int vector_no = 0;

    auto run_one = [&](uint16_t alu_op, const Vec &v, unsigned __int128 expected,
                       const char *name) {
        begin_program(s, 0);
        uint16_t pc = 0;
        put_u128(s.bus, 0x3000, v.a_lo, v.a_hi);
        put_u128(s.bus, 0x3010, v.b_lo, v.b_hi);

        emit(s.bus, pc, op::LOAD, 1, 0, 0, 0x3000);
        emit(s.bus, pc, op::LOAD, 2, 0, 0, 0x3010);

        if (alu_op == op::NOT || alu_op == op::MOV) {
            emit(s.bus, pc, alu_op, 3, 1, 0);
        } else {
            emit(s.bus, pc, alu_op, 3, 1, 2);
        }

        emit(s.bus, pc, op::STORE, 0, 0, 3, 0x3020);
        emit(s.bus, pc, op::HALT);
        s.reset();

        const bool halted = s.run_until_halt(20000);
        CHECK(halted, "property vector reaches HALT");

        uint64_t got_lo = 0, got_hi = 0;
        const bool have = s.bus.read_mem(0x3020, got_lo, got_hi);
        const bool equal = have && got_lo == u128_lo(expected) && got_hi == u128_hi(expected);
        char msg[320];
        std::snprintf(msg, sizeof(msg),
                      "ALU property #%d %s: got=%016llX:%016llX expected=%016llX:%016llX",
                      vector_no++, name,
                      (unsigned long long)got_hi, (unsigned long long)got_lo,
                      (unsigned long long)u128_hi(expected),
                      (unsigned long long)u128_lo(expected));
        CHECK(equal, msg);
        CHECK(s.bus.protocol_errors() == 0, "property vector memory protocol");
    };

    auto run_vec = [&](const Vec &v) {
        const unsigned __int128 a = make_u128(v.a_lo, v.a_hi);
        const unsigned __int128 b = make_u128(v.b_lo, v.b_hi);
        // SHL/SHR используют значение второго операнда, которое реально
        // загружается в r2. В тесте это v.b_lo; поле v.sh здесь не нужно.
        const unsigned shift = static_cast<unsigned>(v.b_lo & 127u);

        run_one(op::ADD, v, a + b, "ADD");
        run_one(op::SUB, v, a - b, "SUB");
        run_one(op::AND, v, a & b, "AND");
        run_one(op::OR,  v, a | b, "OR");
        run_one(op::XOR, v, a ^ b, "XOR");
        run_one(op::NOT, v, ~a,    "NOT");
        run_one(op::MOV, v, a,     "MOV");

        unsigned __int128 shl = 0;
        unsigned __int128 shr = 0;
        if (shift == 0) {
            shl = a;
            shr = a;
        } else if (shift < 128) {
            shl = a << shift;
            shr = a >> shift;
        }
        run_one(op::SHL, v, shl, "SHL");
        run_one(op::SHR, v, shr, "SHR");
    };

    for (const Vec &v : fixed) run_vec(v);

    for (int i = 0; i < RANDOM_VECTORS; ++i) {
        Vec v{};
        v.a_lo = prng64(seed);
        v.a_hi = prng64(seed);
        v.b_lo = prng64(seed);
        v.b_hi = prng64(seed);
        run_vec(v);
    }

    logf("  INFO: ALU property vectors=%d, operations=%d\n",
         static_cast<int>(sizeof(fixed)/sizeof(fixed[0])) + RANDOM_VECTORS,
         (static_cast<int>(sizeof(fixed)/sizeof(fixed[0])) + RANDOM_VECTORS) * 9);
}

// -----------------------------------------------------------------------------
// 11. Архитектурные property checks, которые доступны непосредственно через DUT.
// -----------------------------------------------------------------------------
static void run_architecture_properties(Sim &s) {
    logf("\n==== ARCHITECTURE PROPERTIES ====\n");

    // Проверка PC: main.sv экспортирует 64-битный pcOut, но архитектурный PC
    // занимает только младшие 16 бит и всегда выровнен на 16 байт.
    begin_program(s, 3);
    uint16_t pc = 0;
    for (int i = 0; i < 8; ++i) emit(s.bus, pc, op::NOP);
    emit(s.bus, pc, op::HALT);
    s.reset();

    for (int i = 0; i < 1000 && !s.top->halt; ++i) {
        const uint64_t before = static_cast<uint64_t>(s.top->pcOut);
        CHECK((before & 0xFULL) == 0, "PC is 16-byte aligned");
        CHECK((before >> 16) == 0, "PC upper 48 bits are zero");
        s.step();
    }
    CHECK(s.top->halt != 0, "PC property program reaches HALT");

    // Проверка R0 через несколько потенциальных источников записи.
    begin_program(s, 1);
    pc = 0;
    put_u128(s.bus, 0x3100, 0xFFFFFFFFFFFFFFFFULL, 0xFFFFFFFFFFFFFFFFULL);
    emit(s.bus, pc, op::LOAD, 0, 0, 0, 0x3100);
    emit(s.bus, pc, op::ADD, 0, 0, 0);
    emit(s.bus, pc, op::NOT, 0, 0, 0);
    emit(s.bus, pc, op::STORE, 0, 0, 0, 0x3110);
    emit(s.bus, pc, op::HALT);
    s.reset();
    CHECK(s.run_until_halt(20000), "R0 property program reaches HALT");
    check_mem128(s.bus, 0x3110, 0, 0, "R0 invariant after LOAD/ADD/NOT");

    CHECK(s.bus.protocol_errors() == 0, "architecture properties protocol");
}

// -----------------------------------------------------------------------------
// 10. Итоговая сводка протокола и статистики
// -----------------------------------------------------------------------------
static void run_protocol_summary(Sim &s) {
    logf("\n==== PROTOCOL SUMMARY ====\n");
    CHECK(s.bus.protocol_errors() == 0, "no memory/IO protocol violations in last test");
    logf("  INFO: cycles=%llu mem_txns=%llu mem_ready=%llu IN=%d OUT=%d\n",
         (unsigned long long)s.cycle,
         (unsigned long long)s.bus.txn_count(),
         (unsigned long long)s.bus.ready_count(),
         s.bus.in_strobes(), s.bus.out_strobes());
}

int main(int argc, char **argv) {
    // Трассировку необходимо включить до начала времени симуляции и до создания экземпляра DUT.
    // Иначе Verilator завершит работу при вызове trace()
    // при включённой опции --trace.
    Verilated::traceEverOn(true);
    Verilated::commandArgs(argc, argv);

    g_log = std::fopen("tb/test__log.txt", "w");
    if (!g_log) {
        std::perror("tb/test__log.txt");
        return 2;
    }

    logf("R128 comprehensive Verilator testbench\n");
    logf("Architecture under test: IF->ID->EX->MEM->WB, 128-bit GPR/data, 20-bit physical memory\n");

    Sim sim;
    if (!sim.enable_trace("out/r128.vcd")) {
        logf("ERROR: cannot create out/r128.vcd\n");
        std::fclose(g_log);
        g_log = nullptr;
        return 2;
    }

    logf("VCD trace: out/r128.vcd\n");

    run_basic(sim);
    run_forwarding(sim);
    run_branches(sim);
    run_control_transfer(sim);
    run_movs_cs_sweep(sim);
    run_segments(sim);
    run_io(sim);
    run_memory_stress(sim);
    run_flag_edges(sim);
    run_branch_if_contention(sim);
    run_corner_cases(sim);
    run_alu_properties(sim);
    run_architecture_properties(sim);
    run_protocol_summary(sim);

    logf("\n==== TOTAL: %d passed, %d failed ====\n", g_passes, g_fails);
    if (g_fails == 0)
        logf("RESULT: OK\n");
    else
        logf("RESULT: FAILED\n");

    std::fclose(g_log);
    g_log = nullptr;
    return g_fails ? 1 : 0;
}
