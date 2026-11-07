`default_nettype none
/*------------------------------------------------------------------------------
 * R128 — арифметико-логическое устройство (ALU)
 *
 * Назначение:
 *   Выполняет арифметические, логические операции, сдвиги и сравнения над
 *   128-битными операндами. Выдаёт 128-битный результат и вектор флагов.
 *
 * Интерфейс:
 *   opcode — код операции из r128_pkg;
 *   a, b   — операнды; для сдвигов величина сдвига берётся из b[6:0];
 *   result — результат операции;
 *   flags  — флаги состояния, описанные в r128_pkg.sv.
 *
 * Блок комбинационный: внутреннего состояния и тактового входа нет.
 * Неизвестный opcode помечается flags[4] = 0.
 *----------------------------------------------------------------------------*/
module alu (
    input  logic [15:0]  opcode,
    input  logic [127:0] a,
    input  logic [127:0] b,

    output logic [127:0] result,
    output logic [31:0]  flags
);

    import r128_pkg::*;

    // Для 128-битного операнда достаточно 7 бит: 0..127.
    localparam int SHIFT_WIDTH = 7; // b[6:0] — величина сдвига

    // =====================================================
    // Арифметика, логика и сравнения
    // Все вычисления выполняются над 128-битными значениями. Расширенные
    // 129-битные суммы сохраняют перенос старшего разряда для флага C.
    // Локальные переменные объявлены automatic внутри always_comb.
    // =====================================================

    always_comb begin
        logic [128:0] sum_ext;
        logic [128:0] sub_ext;
        logic [127:0] result_next;
        logic [127:0] flag_result;
        logic [31:0]  flags_next;
        logic         carry_next;
        logic         overflow_next;
        logic         valid_next;
        logic [SHIFT_WIDTH-1:0] shift;
        logic signed [127:0] signed_a;
        logic signed [127:0] signed_b;

        shift    = b[SHIFT_WIDTH-1:0];
        signed_a = $signed(a);
        signed_b = $signed(b);

        // Значения по умолчанию
        result_next   = 128'd0;
        flag_result   = 128'd0;

        sum_ext       = 129'd0;
        sub_ext       = 129'd0;

        carry_next    = 1'b0;
        overflow_next = 1'b0;
        valid_next    = 1'b1;

        // ---------------------------------------------
        // Вычисление результата и промежуточных флагов.
        // ---------------------------------------------

        case (opcode)

            // NOP
            OP_NOP: begin
                result_next = 128'd0;
            end

            // ADD
            OP_ADD: begin

                sum_ext = {1'b0, a} + {1'b0, b};

                result_next = sum_ext[127:0];

                carry_next = sum_ext[128];

                overflow_next =
                    (~(a[127] ^ b[127])) &
                    (result_next[127] ^ a[127]);

            end

            // SUB
            OP_SUB: begin

                sub_ext = {1'b0, a} +
                          {1'b0, ~b} + 129'd1;

                result_next = sub_ext[127:0];

                // При вычитании C=1 означает отсутствие займа
                carry_next = sub_ext[128];

                overflow_next =
                    (a[127] ^ b[127]) &
                    (result_next[127] ^ a[127]);

            end

            // CMP
            // Вычисляет a-b только для формирования флагов.
            // Результат в регистр назначения не записывается.
            OP_CMP: begin

                sub_ext = {1'b0, a} +
                          {1'b0, ~b} + 129'd1;

                flag_result = sub_ext[127:0];

                carry_next = sub_ext[128];

                overflow_next =
                    (a[127] ^ b[127]) &
                    (flag_result[127] ^ a[127]);

                result_next = 128'd0;

            end

            // Логические операции обновляют Z/N, а C/V сбрасывают в 0.
            // Это архитектурная семантика R128 (не сохранение старых флагов).
            // AND
            OP_AND: begin
                result_next = a & b;
                carry_next = 1'b0;
                overflow_next = 1'b0;
            end

            // OR
            OP_OR: begin
                result_next = a | b;
                carry_next = 1'b0;
                overflow_next = 1'b0;
            end

            // XOR
            OP_XOR: begin
                result_next = a ^ b;
                carry_next = 1'b0;
                overflow_next = 1'b0;
            end

            // NOT
            OP_NOT: begin
                result_next = ~a;
                carry_next = 1'b0;
                overflow_next = 1'b0;
            end

            // MOV — копирует a в result.
            // writes_flags(OP_MOV)=0: флаги (в т.ч. C/V по умолчанию 0)
            // не коммитятся в status_flags.
            OP_MOV: begin
                result_next = a;
            end

            // SHL — логический сдвиг влево; carry получает последний
            // вытесненный бит (бит, ушедший за разряд 127).
            // При нулевом сдвиге C = 0. shift = b[SHIFT_WIDTH-1:0].
            OP_SHL: begin
                result_next = a << shift;
                // 128 - shift даёт индекс 127..1 при shift = 1..127
                if (shift != 7'd0)
                    carry_next = a[128 - shift];
            end

            // SHR — логический сдвиг вправо; старшие биты заполняются нулями,
            // carry получает последний вытесненный младший бит.
            // При нулевом сдвиге C = 0.
            OP_SHR: begin
                result_next = a >> shift;
                if (shift != 7'd0)
                    carry_next = a[shift - 1'b1];
            end

            // LOAD и STORE
            // Адрес вычисляется отдельно в основном модуле.
            OP_LOAD,
            OP_STORE: begin
                result_next = 128'd0;
            end

            // JMP и условные переходы
            // Управление PC выполняется в основном модуле.
            OP_JMP,
            OP_JZ,
            OP_JNZ,
            OP_JN,
            OP_JP,
            OP_JC,
            OP_JNC,
            OP_JV,
            OP_JNV,
            OP_JE,
            OP_JNE,
            OP_JUGT,
            OP_JULT,
            OP_JUGE,
            OP_JULE,
            OP_JSGT,
            OP_JSLT,
            OP_JSGE,
            OP_JSLE: begin
                result_next = 128'd0;
            end

            // HALT
            OP_HALT: begin
                result_next = 128'd0;
            end

            // Известные специальные инструкции обрабатываются в main.sv.
            // Здесь они явно считаются допустимыми opcode, хотя ALU-результат
            // для них не используется.
            OP_MOVS,
            OP_MFS,
            OP_LOADS,
            OP_STORES,
            OP_JMPF,
            OP_IN,
            OP_OUT: begin
                result_next = 128'd0;
                valid_next  = 1'b1;
            end

            // Неизвестная операция
            default: begin

                result_next = 128'd0;
                valid_next  = 1'b0;

            end

        endcase

        // ---------------------------------------------
        // Выбор значения для флагов Z и N
        // ---------------------------------------------

        // Для CMP flag_result уже содержит a-b; для остальных операций
        // флаги Z/N формируются по обычному результату ALU.
        if (opcode != OP_CMP)
            flag_result = result_next;

        // =================================================
        // Формирование флагов
        // Z/N вычисляются по результату операции (для CMP — по a-b).
        // EQ/NE и signed/unsigned сравнения вычисляются непосредственно по a,b.
        // =================================================

        flags_next = 32'd0;

        // Основные флаги
        flags_next[0] = (flag_result == 128'd0);
        flags_next[1] = flag_result[127];

        flags_next[2] = carry_next;
        flags_next[3] = overflow_next;

        flags_next[4] = valid_next;

        // ---------------------------------------------
        // Беззнаковые сравнения
        // ---------------------------------------------

        flags_next[5]  = (a == b);
        flags_next[6]  = (a != b);

        flags_next[7]  = (a > b);
        flags_next[8]  = (a < b);
        flags_next[9]  = (a >= b);
        flags_next[10] = (a <= b);

        // ---------------------------------------------
        // Знаковые сравнения
        // ---------------------------------------------

        flags_next[11] = (signed_a > signed_b);
        flags_next[12] = (signed_a < signed_b);
        flags_next[13] = (signed_a >= signed_b);
        flags_next[14] = (signed_a <= signed_b);

        // Выходы ALU — полностью комбинационные. Значения result_next и
        // flags_next рассчитаны только из текущих opcode/a/b.
        result = result_next;
        flags  = flags_next;
    end

endmodule

`default_nettype wire
