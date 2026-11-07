/*------------------------------------------------------------------------------
 * R128 — верхний модуль процессора
 *
 * Назначение:
 *   Объединяет выборку и декодирование инструкций, регистровый файл, ALU,
 *   управление конвейером, сегментацию адресов, интерфейс памяти и I/O.
 *
 * Архитектурные параметры, зафиксированные текущей реализацией:
 *   - ширина данных и регистров GPR: 128 бит;
 *   - длина инструкции: 128 бит, PC увеличивается на 16 байт;
 *   - эффективная ширина PC: 16 бит (старшие 48 бит всегда 0);
 *     адрес выборки и ветвления используют только PC[15:0],
 *     выравнивание на 16-байтную границу (& 16'hFFF0);
 *   - GPR: 64 регистра, R0 всегда читается как ноль;
 *   - физический адрес памяти: 20 бит;
 *   - сегментный адрес: Segment * 16 + Offset;
 *   - конвейер: IF -> ID -> EX -> MEM -> WB;
 *   - интерфейс I/O: 256 портов, каждый шириной 128 бит.
 *
 * Важные соглашения интерфейса:
 *   - memReq обозначает активный запрос общей памяти;
 *   - memRe / memWe задают тип запроса;
 *   - memReady подтверждает завершение текущего запроса;
 *   - memRData содержит валидные данные завершённого чтения в такте
 *     memReady (не X/Z); жёсткая стабильность между тактами не требуется;
 *   - IF принимает инструкцию и увеличивает PC только после memReady;
 *   - LOAD/STORE удерживают EX/MEM до подтверждения memReady;
 *   - HALT в ID/EX ожидает завершения активного LOAD/STORE и IF;
 *   - переходы (branch/JMP/JMPF) ждут !if_fetch_wait && !mem_access;
 *   - IF-транзакция (if_busy) не абортится: при data IF паркуется и
 *     продолжается после data; memRe для IF держится до memReady;
 *   - ioInRead — однотактовый строб при появлении IN в ID/EX (!in_issued),
 *     в том числе во время mem_wait/if_fetch_wait. ioInData фиксируется
 *     в in_data_hold; в EX/MEM идёт hold (или ioInData, если строб и
 *     продвижение в одном такте). Периферии достаточно ответить в такте строба.
 *   - ioOutWrite — однотактовый строб из EX/MEM (!out_issued).
 *----------------------------------------------------------------------------*/
`default_nettype none

module main (
    input  logic         clk,
    input  logic         rst,

    output logic [63:0]  pcOut,
    output logic         halt,

    output logic [19:0]  memAddr,
    output logic [127:0] memWData,
    output logic         memWe,
    output logic         memRe,
    output logic         memReq,
    input  logic         memReady,
    input  logic [127:0] memRData,

    // Отдельный интерфейс ввода-вывода: 256 портов по 128 бит.
    input  logic [127:0] ioInData,
    output logic         ioInRead,
    output logic [7:0]   ioInPort,
    output logic         ioOutWrite,
    output logic [7:0]   ioOutPort,
    output logic [127:0] ioOutData
);

    import r128_pkg::*;

    logic [31:0]  branch_flags;
    logic [31:0]  mem_wb_flags;
    logic         mem_wb_flags_we;

    // Служебное состояние конвейера и однотактовых I/O-стробов.
    // Эти сигналы объявлены до первого использования, что также упрощает
    // анализ проекта инструментами Slang/Yosys.
    logic         out_issued;
    logic         in_issued;
    logic [127:0] in_data_hold; // latched on ioInRead
    logic         mem_wb_valid;
    logic         mem_load;
    logic         mem_store;
    logic         mem_access;
    logic         mem_wait;
    logic         if_busy;
    logic         if_fetch_complete;
    logic         if_fetch_wait;
    // =====================================================
    // IF — выборка инструкции из памяти.
    // PC адресует байты, поэтому следующая 128-битная инструкция находится
    // на PC + 16. Запись PC выполняется только после memReady.
    // =====================================================
    // IF — получение инструкции
    // PC хранится в байтовой адресации. Каждая инструкция занимает 16 байт,
    // поэтому при последовательной выборке PC увеличивается на 16.
    // Архитектурная ширина PC — 16 бит; наружу он расширяется до 64 бит.
    // =====================================================

    logic [15:0] pc;

    // =====================================================
    // Сегментные регистры.
    // CS используется для выборки, DS/SS/ES — для сегментированных данных.
    // =====================================================

    logic [15:0] segment_cs;
    logic [15:0] segment_ds;
    logic [15:0] segment_ss;
    logic [15:0] segment_es;

    logic [19:0] instruction_physical_address;

    assign pcOut = {48'd0, pc};

    // Физический адрес выборки:
    // Physical = make_physical_address(CS, PC).
    // PC уже 16-битный; функция использует его как offset.

    assign instruction_physical_address =
        make_physical_address(segment_cs, {48'd0, pc});

    // =====================================================
    // IF/ID — регистр инструкции
    // =====================================================

    logic [127:0] if_id_instruction;
    logic [15:0]  if_id_pc;
    logic         if_id_valid;

    // =====================================================
    // ID — декодирование инструкции.
    // decoder_inst только выделяет поля; семантика opcode определяется
    // функциями и логикой ниже.
    // =====================================================
    // ID — декодирование инструкции
    // Декодер только выделяет поля; чтение операндов выполняет regfile.
    // Проверка допустимости операции и управление выполнением находятся ниже.
    // =====================================================

    logic [15:0] opcode;
    logic [5:0]  rd;
    logic [5:0]  rs1;
    logic [5:0]  rs2;
    logic [93:0] imm;

    decoder decoder_inst (
        .instruction (if_id_instruction),
        .opcode      (opcode),
        .rd          (rd),
        .rs1         (rs1),
        .rs2         (rs2),
        .imm         (imm)
    );

    // =====================================================
    // REGFILE — банк регистров
    // Два операнда читаются асинхронно; запись результата выполняется
    // синхронно на стадии WB.

    logic         wb_we;
    logic [5:0]   wb_rd;
    logic [127:0] wb_data;

    logic [127:0] reg_data_1;
    logic [127:0] reg_data_2;

    regfile regfile_inst (
        .clk       (clk),
        .rst       (rst),

        .r_we      (wb_we),
        .r_waddr   (wb_rd),
        .r_wdata   (wb_data),

        .r_raddr_1 (rs1),
        .r_raddr_2 (rs2),

        .r_rdata_1 (reg_data_1),
        .r_rdata_2 (reg_data_2)
    );

    // =====================================================
    // ID/EX — граница между декодированием и исполнением.
    // Здесь фиксируются opcode, регистры, операнды, immediate, PC и сегмент,
    // чтобы последующие изменения состояния не меняли уже принятую инструкцию.
    // =====================================================

    logic [15:0]  id_ex_opcode;
    logic [5:0]   id_ex_rd;
    logic [5:0]   id_ex_rs1;
    logic [5:0]   id_ex_rs2;

    logic [127:0] id_ex_a;
    logic [127:0] id_ex_b;

    logic [93:0]  id_ex_imm;
    logic [15:0]  id_ex_pc;

    // Сегмент, зафиксированный для конкретной инструкции в ID/EX.
    // Нельзя читать segment_* напрямую позже в EX/MEM: предыдущий MOVS
    // может коммититься на том же фронте, на котором следующая инструкция
    // входит в ID/EX.
    logic [15:0]  id_ex_segment;

    logic         id_ex_valid;

    // =====================================================
    // EX — forwarding операндов
    // При совпадении номера регистра приоритет имеет более поздняя по времени
    // запись из EX/MEM; если совпадения нет, проверяется MEM/WB. R0 исключён.
    // Для LOAD/LOADS источник EX/MEM должен передавать именно данные памяти.
    // =====================================================

    logic [127:0] ex_a;
    logic [127:0] ex_b;

    logic [127:0] ex_mem_result;
    logic [127:0] ex_mem_load_data;

    logic [127:0] wb_result;

    logic         ex_mem_valid;
    logic [15:0]  ex_mem_opcode;
    logic [5:0]   ex_mem_rd;

    logic         wb_valid;
    logic [15:0]  wb_opcode;
    logic [5:0]   wb_result_rd;

    logic         ex_mem_writes_rd;
    logic         wb_writes_rd;

    logic [127:0] ex_mem_forward_data;
    logic [127:0] wb_forward_data;
    logic         ex_mem_is_load;
    logic         ex_mem_forward_to_a;
    logic         ex_mem_forward_to_b;

    // Метаданные записи берутся из единого описания ISA в r128_pkg.
    always_comb begin
        ex_mem_writes_rd = writes_rd(ex_mem_opcode);
        wb_writes_rd     = writes_rd(wb_opcode);
    end

    // LOAD/LOADS разрешено форвардить только на такте подтверждения памяти.
    // До memReady memRData не определён контрактом интерфейса и может содержать X.
    assign ex_mem_is_load = (ex_mem_opcode == OP_LOAD) ||
                            (ex_mem_opcode == OP_LOADS);

    always_comb begin
        if (ex_mem_is_load)
            ex_mem_forward_data = ex_mem_load_data;
        else
            ex_mem_forward_data = ex_mem_result;
    end

    // Селекторы одновременно учитывают готовность LOAD; при !memReady
    // EX/MEM не может выиграть приоритет у MEM/WB.
    assign ex_mem_forward_to_a =
        ex_mem_valid && ex_mem_writes_rd &&
        (ex_mem_rd != 6'd0) && (ex_mem_rd == id_ex_rs1) &&
        (!ex_mem_is_load || memReady);

    assign ex_mem_forward_to_b =
        ex_mem_valid && ex_mem_writes_rd &&
        (ex_mem_rd != 6'd0) && (ex_mem_rd == id_ex_rs2) &&
        (!ex_mem_is_load || memReady);

    // Данные для forwarding из MEM/WB.

    assign wb_forward_data = wb_result;

    // Выбор операндов с приоритетом EX/MEM → MEM/WB → регистр.
    always_comb begin
        ex_a = id_ex_a;
        if (ex_mem_forward_to_a)
            ex_a = ex_mem_forward_data;
        else if (wb_valid && wb_writes_rd &&
                 (wb_result_rd != 6'd0) && (wb_result_rd == id_ex_rs1))
            ex_a = wb_forward_data;
    end

    always_comb begin
        ex_b = id_ex_b;
        if (ex_mem_forward_to_b)
            ex_b = ex_mem_forward_data;
        else if (wb_valid && wb_writes_rd &&
                 (wb_result_rd != 6'd0) && (wb_result_rd == id_ex_rs2))
            ex_b = wb_forward_data;
    end

    // =====================================================
    // EX — ALU
    // ALU не хранит состояние: результат и флаги вычисляются комбинационно.
    // =====================================================

    logic [127:0] alu_result;
    logic [31:0]  alu_flags;

    alu alu_inst (
        .opcode (id_ex_opcode),
        .a      (ex_a),
        .b      (ex_b),
        .result (alu_result),
        .flags  (alu_flags)
    );

    // =====================================================
    // EX — адрес памяти и переходы
    // =====================================================

    logic [127:0] imm_extended;
    logic [127:0] ex_address;

    logic [19:0]  ex_physical_address;
    logic [15:0]  ex_selected_segment;

    logic         far_jump;
    logic         cs_write_flush;
    logic         ex_instruction_valid;

    logic [15:0]  branch_target;

    logic         branch_taken;

    logic [31:0]  status_flags;

    // Знаковое расширение непосредственного значения.

    assign imm_extended = {
        {34{id_ex_imm[93]}},
        id_ex_imm
    };

    // Адрес LOAD/STORE:
    // Offset = rs1 + sign_extend(imm).

    assign ex_address = ex_a + imm_extended;

    // =====================================================
    // Выбор сегмента для операций памяти / MFS
    //
    // Сегмент фиксируется при входе инструкции в ID/EX. Это важно для
    // конвейера: MOVS предыдущей инструкции коммитится на том же фронте,
    // на котором следующая инструкция может войти в ID/EX.
    //
    // Поэтому есть два уровня:
    //   id_selected_segment — значение для новой инструкции в ID;
    //   id_ex_segment       — захваченное значение для текущей инструкции EX.
    //
    // При совпадении с предыдущим MOVS используется bypass из ex_a.
    // =====================================================

    logic [15:0] id_selected_segment;

    always_comb begin
        case (opcode)
            OP_MFS:
                id_selected_segment = select_segment(
                    rs1[1:0], segment_cs, segment_ds, segment_ss, segment_es);

            OP_LOADS:
                id_selected_segment = select_segment(
                    rs2[1:0], segment_cs, segment_ds, segment_ss, segment_es);

            OP_STORES:
                id_selected_segment = select_segment(
                    rd[1:0], segment_cs, segment_ds, segment_ss, segment_es);

            default:
                // Обычные LOAD/STORE используют DS.
                id_selected_segment = segment_ds;
        endcase

        // Предыдущая инструкция MOVS ещё не успела записать segment_* через
        // NBA. Подменяем только тот сегмент, который она сейчас меняет.
        if (id_ex_valid && (id_ex_opcode == OP_MOVS) && !cs_write_flush) begin
            case (id_ex_rd[1:0])
                SEG_DS:
                    if ((opcode != OP_MFS && opcode != OP_LOADS &&
                         opcode != OP_STORES) ||
                        (opcode == OP_MFS    && rs1[1:0] == SEG_DS) ||
                        (opcode == OP_LOADS  && rs2[1:0] == SEG_DS) ||
                        (opcode == OP_STORES && rd[1:0]  == SEG_DS))
                        id_selected_segment = ex_a[15:0];

                SEG_SS:
                    if ((opcode == OP_MFS    && rs1[1:0] == SEG_SS) ||
                        (opcode == OP_LOADS  && rs2[1:0] == SEG_SS) ||
                        (opcode == OP_STORES && rd[1:0]  == SEG_SS))
                        id_selected_segment = ex_a[15:0];

                SEG_ES:
                    if ((opcode == OP_MFS    && rs1[1:0] == SEG_ES) ||
                        (opcode == OP_LOADS  && rs2[1:0] == SEG_ES) ||
                        (opcode == OP_STORES && rd[1:0]  == SEG_ES))
                        id_selected_segment = ex_a[15:0];

                default: ;
            endcase
        end
    end

    always_comb begin
        ex_selected_segment = id_ex_segment;
    end

    // Физический адрес:
    // Physical = make_physical_address(Segment, Offset).
    // Offset = rs1 + sign_extend(imm); передаём только [19:0]
    // (старшие биты не влияют на 20-битный физический адрес).
    assign ex_physical_address =
        make_physical_address(ex_selected_segment, {44'b0, ex_address[19:0]});

    // =====================================================
    // Специальные инструкции
    // =====================================================

    // ALU уже выставляет flags[4]=1 (valid) для всех известных opcode,
    // включая специальные (MOVS/MFS/LOADS/STORES/JMPF/IN/OUT).
    // OP_NOP намеренно исключается — в EX/MEM уходит bubble, а не valid NOP.
    assign ex_instruction_valid =
        id_ex_valid && valid_opcode(id_ex_opcode) && (id_ex_opcode != OP_NOP);

    // Дальний переход.

    assign far_jump =
        id_ex_valid && (id_ex_opcode == OP_JMPF);

    // MOVS CS требует очистки младших стадий.

    assign cs_write_flush =
        id_ex_valid &&
        (id_ex_opcode == OP_MOVS) &&
        (id_ex_rd[1:0] == SEG_CS);

    // =====================================================
    // Относительный переход
    // =====================================================

    // PC и смещение имеют 16 бит, поэтому арифметика перехода
    // выполняется по модулю 2^16.
    assign branch_target = id_ex_pc + imm_extended[15:0];

    // =====================================================
    // Условие перехода
    // =====================================================

    always_comb begin

        branch_taken = 1'b0;

        if (id_ex_valid) begin

            if ((id_ex_opcode == OP_JMP) || far_jump)
                branch_taken = 1'b1;

            else if (branch_flags[4])
                branch_taken =
                    branch_condition(id_ex_opcode, branch_flags);

        end

    end

    // =====================================================
    // EX/MEM — регистры данных для памяти и ввода-вывода
    // Объявления расположены до непрерывных присваиваний, которые их используют.
    // =====================================================

    logic [19:0]  ex_mem_address;
    logic [127:0] ex_mem_store_data;
    logic [7:0]   ex_mem_io_port;
    logic [31:0]  ex_mem_flags;
    logic         ex_mem_flags_we;


    // Флаговый forwarding: самый новый готовый результат имеет приоритет.
    // В такте T mem_wb_flags содержит флаги инструкции, которая
    // коммитится в status_flags на фронте T; поэтому forwarding из MEM/WB
    // корректен для branch_condition, вычисляемого до этого фронта.
    always_comb begin
        branch_flags = status_flags;
        if (mem_wb_valid && mem_wb_flags_we)
            branch_flags = mem_wb_flags;
        if (ex_mem_valid && ex_mem_flags_we)
            branch_flags = ex_mem_flags;
    end

    // =====================================================
    // Интерфейс ввода-вывода.
    // IN выдаёт один строб и фиксирует входные данные; OUT выдаёт один строб
    // на удерживаемую инструкцию, чтобы ожидание памяти не создавало повторов.
    // =====================================================

    // IN: строб один раз на инструкцию; данные → in_data_hold.
    assign ioInRead = id_ex_valid && (id_ex_opcode == OP_IN) && !in_issued;
    assign ioInPort = id_ex_imm[7:0];

    always_ff @(posedge clk) begin
        if (rst) begin
            in_issued    <= 1'b0;
            in_data_hold <= 128'd0;
        end else if (ioInRead) begin
            in_issued    <= 1'b1;
            in_data_hold <= ioInData;
        end else if (!id_ex_valid || (id_ex_opcode != OP_IN)) begin
            // IN ушёл из ID/EX — разрешаем строб следующей IN-инструкции
            in_issued <= 1'b0;
        end
    end

    // Однотактовый OUT-строб: при удержании EX/MEM во время ожидания
    // запроса IF/MEM событие не повторяется.
    assign ioOutWrite = ex_mem_valid &&
                        (ex_mem_opcode == OP_OUT) &&
                        !out_issued;

    // Сброс out_issued безопасен при !mem_wait && !if_fetch_wait:
    // в том же такте EX/MEM продвигается или очищается.
    always_ff @(posedge clk) begin
        if (rst)
            out_issued <= 1'b0;
        else if (!mem_wait && !if_fetch_wait)
            out_issued <= 1'b0;
        else if (ioOutWrite)
            out_issued <= 1'b1;
    end
    assign ioOutPort  = ex_mem_io_port;
    assign ioOutData  = ex_mem_store_data;

    // =====================================================
    // MEM — доступ к памяти.
    // mem_bus объединяет IF и data side на одном внешнем порте и удерживает
    // запрос до memReady.
    // LOAD и STORE используют общий внешний интерфейс. Адрес уже преобразован
    // в 20-битный физический адрес; данные чтения приходят через memRData.
    // Синхронизация с внешней памятью определяется контрактом верхнего уровня.
    // =====================================================

    assign mem_load =
        ex_mem_valid &&
        ((ex_mem_opcode == OP_LOAD) || (ex_mem_opcode == OP_LOADS));

    assign mem_store =
        ex_mem_valid &&
        ((ex_mem_opcode == OP_STORE) || (ex_mem_opcode == OP_STORES));

    // Арбитр IF vs data: IF не абортится mid-flight.
    mem_bus mem_bus_inst (
        .clk          (clk),
        .rst          (rst),
        .halt         (halt),
        .branch_taken (branch_taken),
        .mem_load     (mem_load),
        .mem_store    (mem_store),
        .data_addr    (ex_mem_address),
        .data_wdata   (ex_mem_store_data),
        .if_addr      (instruction_physical_address),
        .memReady     (memReady),
        .memAddr      (memAddr),
        .memWData     (memWData),
        .memWe        (memWe),
        .memRe        (memRe),
        .memReq       (memReq),
        .mem_access   (mem_access),
        .mem_wait     (mem_wait),
        .if_busy      (if_busy),
        .if_fetch_wait    (if_fetch_wait),
        .if_fetch_complete(if_fetch_complete)
    );

    assign ex_mem_load_data = memRData;

    // =====================================================
    // MEM/WB — граница MEM → WB.
    // Для LOAD результат берётся из memRData только на подтверждённом такте;
    // для остальных инструкций передаётся результат EX.
    // =====================================================
    // MEM/WB — регистр результата для записи
    // =====================================================

    logic [127:0] mem_wb_result;
    logic [5:0]   mem_wb_rd;

    logic [15:0]  mem_wb_opcode;
    // mem_wb_valid declared near module top

    assign wb_we =
        wb_valid &&
        wb_writes_rd &&
        (wb_result_rd != 6'd0);

    assign wb_rd   = wb_result_rd;
    assign wb_data = wb_result;

    // =====================================================
    // WB — подготовка результата.
    // wb_we становится активным только для валидной инструкции, которая
    // действительно записывает GPR и не использует архитектурный R0.
    // =====================================================

    assign wb_valid     = mem_wb_valid;
    assign wb_result    = mem_wb_result;
    assign wb_result_rd = mem_wb_rd;
    assign wb_opcode    = mem_wb_opcode;

    always_ff @(posedge clk) begin

        if (rst) begin

            mem_wb_result <= 128'd0;
            mem_wb_rd     <= 6'd0;
            mem_wb_opcode   <= OP_NOP;
            mem_wb_valid    <= 1'b0;
            mem_wb_flags    <= 32'd0;
            mem_wb_flags_we <= 1'b0;

        end

        else begin

            // LOAD/STORE попадают в WB только на такте подтверждения.
            // При ожидании выдаётся bubble, чтобы результат не фиксировался повторно.
            mem_wb_valid <= ex_mem_valid && (!mem_access || memReady);

            if (ex_mem_valid && (!mem_access || memReady)) begin
                mem_wb_flags    <= ex_mem_flags;
                mem_wb_flags_we <= ex_mem_flags_we;

                mem_wb_rd     <= ex_mem_rd;
                mem_wb_opcode <= ex_mem_opcode;

                if ((ex_mem_opcode == OP_LOAD) ||
                    (ex_mem_opcode == OP_LOADS))
                    mem_wb_result <= memRData;

                else
                    mem_wb_result <= ex_mem_result;

            end

            else begin
                // Bubble: обнуляем все поля, включая result.
                mem_wb_result   <= 128'd0;
                mem_wb_rd       <= 6'd0;
                mem_wb_opcode   <= OP_NOP;
                mem_wb_flags    <= 32'd0;
                mem_wb_flags_we <= 1'b0;

            end

        end

    end

    // =====================================================
    // WB — архитектурная фиксация флагов
    // Флаги становятся видимыми архитектуре только при commit в WB.
    // =====================================================

    always_ff @(posedge clk) begin
        if (rst)
            status_flags <= 32'd0;
        else if (mem_wb_valid && mem_wb_flags_we)
            status_flags <= mem_wb_flags;
    end

    // =====================================================
    // HALT — остановка процессора.
    // Остановка видна наружу только после того, как завершены уже начатые
    // обращения к памяти, иначе можно оборвать действующий memReq.
    // =====================================================

    always_ff @(posedge clk) begin

        if (rst)
            halt <= 1'b0;

        // Не останавливаем процессор, пока предыдущий LOAD/STORE
        // в EX/MEM не завершил транзакцию памяти.
        // HALT нельзя завершать, пока не подтверждён уже начатый IF-запрос.
        // Иначе halt отключит memRe до memReady и нарушит протокол памяти.
        else if (id_ex_valid &&
                 (id_ex_opcode == OP_HALT) &&
                 !mem_access &&
                 !if_fetch_wait)
            halt <= 1'b1;

    end

    // =====================================================
    // IF/ID и ID/EX — управление конвейером.
    // Приоритет: reset → halt/flush → переход → смена CS → stall → обычное
    // продвижение. Это не даёт младшим стадиям обойти управляющее событие.
    // =====================================================

    // Общие действия очистки стадий (только NBA — безопасно из always_ff).
    task automatic clear_if_id;
        if_id_instruction <= 128'd0;
        if_id_pc          <= 16'd0;
        if_id_valid       <= 1'b0;
    endtask

    task automatic clear_id_ex;
        id_ex_opcode <= OP_NOP;
        id_ex_rd     <= 6'd0;
        id_ex_rs1    <= 6'd0;
        id_ex_rs2    <= 6'd0;
        id_ex_a      <= 128'd0;
        id_ex_b      <= 128'd0;
        id_ex_imm    <= 94'd0;
        id_ex_pc     <= 16'd0;
        id_ex_segment <= 16'd0;
        id_ex_valid  <= 1'b0;
    endtask

    task automatic flush_front;
        clear_if_id();
        clear_id_ex();
    endtask

    always_ff @(posedge clk) begin
        if (rst) begin
            pc         <= 16'd0;
            segment_cs <= 16'd0;
            segment_ds <= 16'd0;
            segment_ss <= 16'd0;
            segment_es <= 16'd0;
            flush_front();
        end else begin
            // -------------------------------------------------
            // Side-effect: MOVS DS/SS/ES (не flush)
            // -------------------------------------------------
            if (id_ex_valid &&
                (id_ex_opcode == OP_MOVS) &&
                !cs_write_flush &&
                !branch_taken &&
                !mem_access &&
                !if_fetch_wait) begin
                case (id_ex_rd[1:0])
                    SEG_DS: segment_ds <= ex_a[15:0];
                    SEG_SS: segment_ss <= ex_a[15:0];
                    SEG_ES: segment_es <= ex_a[15:0];
                    default: /* CS handled below */ ;
                endcase
            end

            // -------------------------------------------------
            // Приоритетный контроль стадий IF/ID и ID/EX
            // -------------------------------------------------
            if (halt) begin
                flush_front();
            end
            // Переход: ждём завершения IF и отсутствия data-access.
            else if (branch_taken && !if_fetch_wait && !mem_access) begin
                if (far_jump) begin
                    segment_cs <= id_ex_imm[15:0];
                    pc <= (id_ex_imm[31:16] & 16'hFFF0);
                end else begin
                    pc <= (branch_target & 16'hFFF0);
                end
                flush_front();
            end
            // MOVS CS: смена сегмента кода + flush младших стадий.
            else if (cs_write_flush && !branch_taken &&
                     !mem_access && !if_fetch_wait) begin
                segment_cs <= ex_a[15:0];
                flush_front();
            end
            // HALT в ID/EX: flush после завершения IF.
            else if (id_ex_valid && (id_ex_opcode == OP_HALT) &&
                     !mem_access && !if_fetch_wait) begin
                flush_front();
            end
            // Stall: data mem или IF — удерживаем ID/EX, фиксируем fwd-операнды.
            else if (mem_access || if_fetch_wait) begin
                id_ex_a <= ex_a;
                id_ex_b <= ex_b;
            end
            // -------------------------------------------------
            // Нормальное продвижение конвейера
            // -------------------------------------------------
            else begin
                id_ex_valid <= if_id_valid;

                if (if_id_valid) begin
                    id_ex_opcode <= opcode;
                    id_ex_rd     <= rd;
                    id_ex_rs1    <= rs1;
                    id_ex_rs2    <= rs2;

                    // WB → ID bypass
                    id_ex_a <= (wb_we && (wb_rd != 6'd0) && (wb_rd == rs1))
                               ? wb_data : reg_data_1;
                    id_ex_b <= (wb_we && (wb_rd != 6'd0) && (wb_rd == rs2))
                               ? wb_data : reg_data_2;

                    id_ex_imm     <= imm;
                    id_ex_pc      <= if_id_pc;
                    id_ex_segment <= id_selected_segment;
                end else begin
                    clear_id_ex();
                end

                // IF/ID: принять инструкцию только после memReady.
                if (if_fetch_complete) begin
                    if_id_instruction <= memRData;
                    if_id_pc          <= pc;
                    if_id_valid       <= 1'b1;
                    pc <= pc + 16'd16;
                end else begin
                    if_id_valid <= 1'b0;
                end
            end
        end
    end

    // =====================================================
    // EX/MEM — граница EX → MEM.
    // Здесь удерживаются адрес, данные STORE, результат ALU, I/O-порт
    // и флаги до тех пор, пока внешняя память не завершит транзакцию.
    // =====================================================
    // EX/MEM — синхронная регистрация результата
    // =====================================================

    task automatic clear_ex_mem;
        ex_mem_result     <= 128'd0;
        ex_mem_address    <= 20'd0;
        ex_mem_store_data <= 128'd0;
        ex_mem_io_port    <= 8'd0;
        ex_mem_flags      <= 32'd0;
        ex_mem_flags_we   <= 1'b0;
        ex_mem_rd         <= 6'd0;
        ex_mem_opcode     <= OP_NOP;
        ex_mem_valid      <= 1'b0;
    endtask

    always_ff @(posedge clk) begin
        if (rst) begin
            clear_ex_mem();
        end
        // Stall: держим EX/MEM неизменным.
        else if (mem_wait || if_fetch_wait) begin
            // mem_wait  — data-транзакция ждёт memReady
            // if_fetch_wait — IF владеет шиной, не выпускаем ID/EX
        end
        // Завершённый data-access → bubble на один такт.
        else if (mem_access && memReady) begin
            clear_ex_mem();
        end
        else begin
            ex_mem_valid <= ex_instruction_valid;

            if (ex_instruction_valid) begin
                // Результат: IN / MFS / ALU
                // Если строб в этом же такте — берём ioInData (NBA hold ещё старый);
                // если строб был раньше при stall — берём in_data_hold.
                if (id_ex_opcode == OP_IN)
                    ex_mem_result <= ioInRead ? ioInData : in_data_hold;
                else if (id_ex_opcode == OP_MFS)
                    ex_mem_result <= {112'd0, id_ex_segment};
                else
                    ex_mem_result <= alu_result;

                ex_mem_address <= ex_physical_address;
                ex_mem_store_data <= (id_ex_opcode == OP_OUT) ? ex_a : ex_b;
                ex_mem_io_port    <= id_ex_imm[7:0];

                if (writes_flags(id_ex_opcode)) begin
                    ex_mem_flags    <= alu_flags;
                    ex_mem_flags_we <= 1'b1;
                end else begin
                    ex_mem_flags    <= 32'd0;
                    ex_mem_flags_we <= 1'b0;
                end

                ex_mem_rd     <= id_ex_rd;
                ex_mem_opcode <= id_ex_opcode;
            end else begin
                clear_ex_mem();
            end
        end
    end

endmodule

`default_nettype wire
