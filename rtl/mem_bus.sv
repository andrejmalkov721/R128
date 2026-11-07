`default_nettype none
/*------------------------------------------------------------------------------
 * R128 — арбитр общей шины памяти (IF vs data)
 *
 * Правила:
 *   1. Data (LOAD/STORE) имеет приоритет на комбинационном mux.
 *   2. Начатый IF (if_busy) не отменяется: ждёт memReady, даже если
 *      branch_taken стал 1. При появлении data IF «паркуется» (if_busy
 *      остаётся 1), memRe отдаётся data; после data IF продолжается.
 *   3. Новый IF стартует только при !halt && !branch_taken && !mem_access
 *      && !if_busy.
 *
 * Контракт: memRe/memWe/memAddr/memWData стабильны, пока memReq && !memReady.
 *----------------------------------------------------------------------------*/
module mem_bus (
    input  logic         clk,
    input  logic         rst,

    // Состояние конвейера: при halt новые IF-запросы не начинаются,
    // а branch_taken запрещает запуск нового IF до применения перехода.
    input  logic         halt,
    input  logic         branch_taken,

    // Data side (EX/MEM)
    input  logic         mem_load,
    input  logic         mem_store,
    input  logic [19:0]  data_addr,
    input  logic [127:0] data_wdata,

    // IF side
    input  logic [19:0]  if_addr,

    // Общий внешний порт памяти. В каждый момент времени на шине
    // находится либо data-транзакция, либо IF-транзакция.
    input  logic         memReady,
    output logic [19:0]  memAddr,
    output logic [127:0] memWData,
    output logic         memWe,
    output logic         memRe,
    output logic         memReq,

    // Статус для конвейера: позволяет main.sv удерживать EX/MEM
    // и не продвигать зависимые стадии до завершения запроса.
    output logic         mem_access,
    output logic         mem_wait,
    output logic         if_busy,
    output logic         if_fetch_wait,
    output logic         if_fetch_complete
);

    // Data-запрос считается активным независимо от memReady; mem_wait
    // дополнительно показывает, что такой запрос ещё не завершён.
    assign mem_access = mem_load || mem_store;
    assign mem_wait   = mem_access && !memReady;

    // -------------------------------------------------------------------------
    // Sticky IF busy: незавершённая выборка не отменяется.
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            if_busy <= 1'b0;
        end else if (if_busy && memReady && !mem_access) begin
            // IF completed (шина была у IF в этом такте)
            if_busy <= 1'b0;
        end else if (!if_busy && !halt && !branch_taken && !mem_access) begin
            if_busy <= 1'b1;
        end
        // mem_access && if_busy: data забирает шину, if_busy сохраняется —
        // после data IF будет продолжен с тем же if_addr.
    end

    assign if_fetch_wait    = if_busy && !memReady && !mem_access;
    assign if_fetch_complete = if_busy && memReady && !mem_access;

    // -------------------------------------------------------------------------
    // Арбитраж шины: data имеет приоритет над IF.
    // Если data нет, активная IF-транзакция продолжает использовать шину.
    // -------------------------------------------------------------------------
    assign memReq = (!rst) && (memRe || memWe);

    always_comb begin
        memAddr  = if_addr;
        memWData = 128'd0;
        memWe    = 1'b0;
        memRe    = 1'b0;

        if (!rst) begin
            if (mem_load) begin
                memAddr = data_addr;
                memRe   = 1'b1;
            end else if (mem_store) begin
                memAddr  = data_addr;
                memWData = data_wdata;
                memWe    = 1'b1;
            end else if (if_busy) begin
                memAddr = if_addr;
                memRe   = 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
