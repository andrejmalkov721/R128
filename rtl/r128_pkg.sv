`default_nettype none
/*------------------------------------------------------------------------------
 * R128 — общие определения архитектуры
 *
 * Содержит единый источник определений, используемых RTL-модулями:
 *   - коды операций (opcode);
 *   - селекторы сегментных регистров;
 *   - функции построения физического адреса и выбора сегмента;
 *   - декодирование условий перехода по вектору флагов;
 *   - метаданные ISA: какие инструкции допустимы и какие архитектурные
 *     состояния они изменяют.
 *----------------------------------------------------------------------------*/
package r128_pkg;

    // =====================================================
    // Коды операций
    // Значения opcode являются частью бинарного интерфейса ISA. Не менять
    // существующие значения без обновления программ, тестов и декодирования.
    // =====================================================

    localparam logic [15:0] OP_NOP   = 16'h0000;
    localparam logic [15:0] OP_ADD   = 16'h0001;
    localparam logic [15:0] OP_SUB   = 16'h0002;
    localparam logic [15:0] OP_AND   = 16'h0003;
    localparam logic [15:0] OP_OR    = 16'h0004;
    localparam logic [15:0] OP_XOR   = 16'h0005;
    localparam logic [15:0] OP_NOT   = 16'h0006;
    localparam logic [15:0] OP_SHL   = 16'h0007;
    localparam logic [15:0] OP_SHR   = 16'h0008;
    localparam logic [15:0] OP_MOV   = 16'h0009;
    localparam logic [15:0] OP_LOAD  = 16'h000A;
    localparam logic [15:0] OP_STORE = 16'h000B;
    localparam logic [15:0] OP_JMP   = 16'h000C;
    localparam logic [15:0] OP_HALT  = 16'h000D;

    // =====================================================
    // Условные переходы
    // =====================================================

    localparam logic [15:0] OP_JZ   = 16'h000E;
    localparam logic [15:0] OP_JNZ  = 16'h000F;
    localparam logic [15:0] OP_JN   = 16'h0010;
    localparam logic [15:0] OP_JP   = 16'h0011;
    localparam logic [15:0] OP_JC   = 16'h0012;
    localparam logic [15:0] OP_JNC  = 16'h0013;
    localparam logic [15:0] OP_JV   = 16'h0014;
    localparam logic [15:0] OP_JNV  = 16'h0015;
    localparam logic [15:0] OP_JE   = 16'h0016;
    localparam logic [15:0] OP_JNE  = 16'h0017;

    localparam logic [15:0] OP_JUGT = 16'h0018;
    localparam logic [15:0] OP_JULT = 16'h0019;
    localparam logic [15:0] OP_JUGE = 16'h001A;
    localparam logic [15:0] OP_JULE = 16'h001B;

    localparam logic [15:0] OP_JSGT = 16'h001C;
    localparam logic [15:0] OP_JSLT = 16'h001D;
    localparam logic [15:0] OP_JSGE = 16'h001E;
    localparam logic [15:0] OP_JSLE = 16'h001F;

    // =====================================================
    // Сравнение
    // =====================================================

    localparam logic [15:0] OP_CMP = 16'h0020;

    // =====================================================
    // Сегментные инструкции
    // =====================================================

    localparam logic [15:0] OP_MOVS  = 16'h0021;
    localparam logic [15:0] OP_MFS   = 16'h0022;
    localparam logic [15:0] OP_LOADS = 16'h0023;
    localparam logic [15:0] OP_STORES= 16'h0024;
    localparam logic [15:0] OP_JMPF  = 16'h0025;

    // =====================================================
    // Инструкции ввода-вывода
    // =====================================================

    localparam logic [15:0] OP_IN  = 16'h0026;
    localparam logic [15:0] OP_OUT = 16'h0027;

    // =====================================================
    // Идентификаторы сегментов
    // =====================================================

    localparam logic [1:0] SEG_CS = 2'b00;
    localparam logic [1:0] SEG_DS = 2'b01;
    localparam logic [1:0] SEG_SS = 2'b10;
    localparam logic [1:0] SEG_ES = 2'b11;

    // =====================================================
    // Формирование физического адреса
    //
    // Physical = (Segment << 4) + Offset[19:0]
    //
    // Segment: 16 бит
    // Offset:  используется младшие 20 бит (старшие отбрасываются)
    // Address: 20 бит (переполнение по модулю 2^20)
    //
    // В текущей реализации PC и эффективный offset для
    // LOAD/STORE ограничены 16 битами из-за адресации
    // и выравнивания инструкций, но функция принимает
    // полный 64-битный offset для единообразия.
    // =====================================================

    function automatic logic [19:0] make_physical_address(
        input logic [15:0] segment,
        input logic [63:0] offset
    );

        logic [19:0] segment_base;

        begin
            segment_base = {segment, 4'b0000};

            make_physical_address =
                segment_base + offset[19:0];
        end

    endfunction

    // =====================================================
    // Выбор сегмента
    // =====================================================

    function automatic logic [15:0] select_segment(
        input logic [1:0] selector,
        input logic [15:0] cs,
        input logic [15:0] ds,
        input logic [15:0] ss,
        input logic [15:0] es
    );

        begin
            case (selector)

                SEG_CS: select_segment = cs;
                SEG_DS: select_segment = ds;
                SEG_SS: select_segment = ss;
                SEG_ES: select_segment = es;

                default: select_segment = ds;

            endcase
        end

    endfunction

    // =====================================================
    // Формат фиксированной 128-битной инструкции.
    // Этот формат используется decoder.sv и должен совпадать с кодером ПО.
    // =====================================================
    // Формат инструкции
    //
    // [127:112] opcode
    // [111:106] rd
    // [105:100] rs1
    // [99:94]   rs2
    // [93:0]    imm
    // =====================================================

    // =====================================================
    // Формат сегментных инструкций
    // Примечание по JMPF: в текущем RTL main.sv используются imm[15:0] для
    // нового CS и imm[31:16] для нового PC (16-битное смещение).
    //
    // MOVS:
    //   rd[1:0] = номер сегмента
    //   rs1     = GPR-источник
    //
    // MFS:
    //   rd      = GPR-приёмник
    //   rs1[1:0]= номер сегмента
    //
    // LOADS:
    //   rd      = GPR-приёмник
    //   rs1     = GPR-база
    //   rs2[1:0]= номер сегмента
    //   imm     = смещение
    //
    // STORES:
    //   rd[1:0] = номер сегмента
    //   rs1     = GPR-база
    //   rs2     = GPR-источник данных
    //   imm     = смещение
    //
    // JMPF:
    //   imm[15:0]  = новый CS
    //   imm[31:16] = новый PC (16-битное смещение)
    // =====================================================

    // =====================================================
    // Формат флагов ALU
    // =====================================================
    //
    // flags[0]  Z
    // flags[1]  N
    // flags[2]  C
    // flags[3]  V
    // Семантика флагов: ADD/SUB/CMP вычисляют C и V; SHL/SHR вычисляют C
    // и сбрасывают V; AND/OR/XOR/NOT обновляют Z/N и явно сбрасывают C/V.
    // flags[4]  VALID
    //
    // flags[5]  EQ
    // flags[6]  NE
    // flags[7]  UGT
    // flags[8]  ULT
    // flags[9]  UGE
    // flags[10] ULE
    // flags[11] SGT
    // flags[12] SLT
    // flags[13] SGE
    // flags[14] SLE
    //
    // Важно: flags[5..14] (сравнения) всегда вычисляются по операндам
    // a и b текущего цикла ALU, а не по result. Они архитектурно
    // корректны и имеют смысл только сразу после CMP (и при
    // writes_flags=1). Для остальных операций эти биты в status_flags
    // не коммитятся (writes_flags=0).
    // =====================================================

    function automatic logic branch_condition(
        input logic [15:0] opcode,
        input logic [31:0] flags
    );

        begin
            case (opcode)

                OP_JZ:   branch_condition =  flags[0];
                OP_JNZ:  branch_condition = ~flags[0];

                OP_JN:   branch_condition =  flags[1];
                OP_JP:   branch_condition = ~flags[1];

                OP_JC:   branch_condition =  flags[2];
                OP_JNC:  branch_condition = ~flags[2];

                OP_JV:   branch_condition =  flags[3];
                OP_JNV:  branch_condition = ~flags[3];

                OP_JE:   branch_condition =  flags[5];
                OP_JNE:  branch_condition =  flags[6];

                OP_JUGT: branch_condition =  flags[7];
                OP_JULT: branch_condition =  flags[8];
                OP_JUGE: branch_condition =  flags[9];
                OP_JULE: branch_condition =  flags[10];

                OP_JSGT: branch_condition =  flags[11];
                OP_JSLT: branch_condition =  flags[12];
                OP_JSGE: branch_condition =  flags[13];
                OP_JSLE: branch_condition =  flags[14];

                default: branch_condition = 1'b0;

            endcase
        end

    endfunction


    // =====================================================
    // Метаданные ISA: единый источник истины для конвейера.
    // =====================================================

    // Возвращает 1 для любого opcode, который архитектура R128
    // определяет как существующую инструкцию. Проверка допустимости
    // opcode не должна зависеть от ALU: специальные инструкции
    // (MOVS/MFS/LOADS/STORES/JMPF/IN/OUT) не являются ALU-операциями.
    function automatic logic valid_opcode(input logic [15:0] op);
        begin
            case (op)
                OP_NOP,
                OP_ADD, OP_SUB, OP_AND, OP_OR, OP_XOR, OP_NOT,
                OP_SHL, OP_SHR, OP_MOV, OP_LOAD, OP_STORE, OP_JMP,
                OP_HALT,
                OP_JZ, OP_JNZ, OP_JN, OP_JP, OP_JC, OP_JNC,
                OP_JV, OP_JNV, OP_JE, OP_JNE,
                OP_JUGT, OP_JULT, OP_JUGE, OP_JULE,
                OP_JSGT, OP_JSLT, OP_JSGE, OP_JSLE,
                OP_CMP,
                OP_MOVS, OP_MFS, OP_LOADS, OP_STORES, OP_JMPF,
                OP_IN, OP_OUT:
                    valid_opcode = 1'b1;
                default:
                    valid_opcode = 1'b0;
            endcase
        end
    endfunction

    // Возвращает 1, если инструкция создаёт значение, которое должно
    // попасть в GPR через стадию WB. Store/branch/OUT/HALT сюда не входят.
    function automatic logic writes_rd(input logic [15:0] op);
        begin
            case (op)
                OP_ADD, OP_SUB, OP_AND, OP_OR, OP_XOR,
                OP_NOT, OP_SHL, OP_SHR, OP_MOV,
                OP_LOAD, OP_MFS, OP_LOADS, OP_IN:
                    writes_rd = 1'b1;
                default:
                    writes_rd = 1'b0;
            endcase
        end
    endfunction

    // Возвращает 1, если результат ALU и его флаги должны быть переданы
    // до WB и затем зафиксированы в status_flags.
    function automatic logic writes_flags(input logic [15:0] op);
        begin
            case (op)
                OP_ADD, OP_SUB, OP_AND, OP_OR, OP_XOR,
                OP_NOT, OP_SHL, OP_SHR, OP_CMP:
                    writes_flags = 1'b1;
                default:
                    writes_flags = 1'b0;
            endcase
        end
    endfunction

endpackage

`default_nettype wire
