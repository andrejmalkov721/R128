#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
R128 Assembler — однофайловый ассемблер для учебного процессора R128.

Назначение:
  Преобразует текстовый исходный код на языке ассемблера R128 в бинарный
  образ памяти (raw binary), пригодный для загрузки в симулятор / testbench
  или в память процессора.

Архитектурные параметры, зафиксированные текущей реализацией RTL:
  - ширина данных и регистров GPR: 128 бит;
  - длина инструкции: 128 бит = 16 байт, PC увеличивается на 16;
  - эффективная ширина PC: 16 бит (старшие 48 бит всегда 0);
  - GPR: 64 регистра (r0..r63), r0 всегда читается как ноль;
  - физический адрес памяти: 20 бит;
  - сегментный адрес: Segment * 16 + Offset;
  - формат инструкции (поля в big-endian порядке бит, байты в памяти little-endian):
        [127:112] opcode   — 16 бит
        [111:106] rd       — 6 бит
        [105:100] rs1      — 6 бит
        [ 99: 94] rs2      — 6 бит
        [ 93:  0] imm      — 94 бита

Важные соглашения кодирования (должны совпадать с RTL):
  - Относительные переходы:
        target = PC + sign_extend(imm)
    где PC — адрес текущей инструкции. Ассемблер вычисляет
        imm = label - текущий_PC
    (а не label - (PC + 16)).
  - STORE:  rs1 = base, rs2 = данные для записи.
  - MOVS:   rd[1:0] = селектор сегмента, rs1 = источник.
  - MFS:    rd = назначение, rs1[1:0] = селектор сегмента.
  - LOADS:  rd = назначение, rs1 = base, rs2[1:0] = сегмент, imm = смещение.
  - STORES: rd[1:0] = сегмент, rs1 = base, rs2 = данные, imm = смещение.
  - JMPF:   imm[15:0] = CS, imm[31:16] = новый PC.
  - IN/OUT: младшие 8 бит imm содержат номер порта (0..255).

Возможности ассемблера:
  - двухпроходная сборка с метками;
  - директивы .org, .align, .byte, .word, .word128, .fill;
  - поддержка всех opcode из rtl/r128_pkg.sv;
  - понятные сообщения об ошибках с номером строки;
  - опциональный listing и hex-dump.

Примеры синтаксиса:

    .org 0x0000

    start:
        mov   r1, r0
        load  r2, r0, 0x0200
        add   r3, r1, r2
        store r3, r0, 0x0210
        cmp   r3, r2
        je    equal
        halt

    equal:
        mov   r4, r3
        halt

Сегменты:

    movs ds, r1
    movs ss, r2
    movs es, r3
    movs cs, r4

    mfs  r5, cs
    mfs  r6, ds

    loads  r7, r0, ds, 0x1000
    stores ds, r0, r7, 0x1010

Дальний переход:

    jmpf  1, 0x0040
    # или
    jmpf  0x0001, 0x0040

I/O:

    in   r1, 0x10
    out  r1, 0x20

Запуск:

    python3 asm.py program.asm -o program.bin
    python3 asm.py program.asm -o program.bin --listing program.lst
    python3 asm.py program.asm --hex
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


# =============================================================================
# ISA — коды операций и селекторы сегментов
#
# Значения opcode являются частью бинарного интерфейса ISA.
# Они должны в точности совпадать с localparam в rtl/r128_pkg.sv.
# Изменение существующего opcode без одновременного обновления RTL,
# testbench и программ приводит к несовместимости.
# =============================================================================

OP = {
    # Базовые операции
    "nop":   0x0000,
    "add":   0x0001,
    "sub":   0x0002,
    "and":   0x0003,
    "or":    0x0004,
    "xor":   0x0005,
    "not":   0x0006,
    "shl":   0x0007,
    "shr":   0x0008,
    "mov":   0x0009,
    "load":  0x000A,
    "store": 0x000B,
    "jmp":   0x000C,
    "halt":  0x000D,

    # Условные переходы (флаги Z/N/C/V и производные)
    "jz":    0x000E,
    "jnz":   0x000F,
    "jn":    0x0010,
    "jp":    0x0011,
    "jc":    0x0012,
    "jnc":   0x0013,
    "jv":    0x0014,
    "jnv":   0x0015,
    "je":    0x0016,
    "jne":   0x0017,

    # Беззнаковые сравнения
    "jugt":  0x0018,
    "jult":  0x0019,
    "juge":  0x001A,
    "jule":  0x001B,

    # Знаковые сравнения
    "jsgt":  0x001C,
    "jslt":  0x001D,
    "jsge":  0x001E,
    "jsle":  0x001F,

    # Сравнение (устанавливает флаги, результат не пишется в регистр)
    "cmp":   0x0020,

    # Сегментные инструкции
    "movs":  0x0021,   # запись в сегментный регистр
    "mfs":   0x0022,   # чтение сегментного регистра
    "loads": 0x0023,   # load с явным сегментом
    "stores":0x0024,   # store с явным сегментом
    "jmpf":  0x0025,   # дальний переход (CS + PC)

    # Ввод-вывод
    "in":    0x0026,
    "out":   0x0027,
}

# Селекторы сегментных регистров (совпадают с SEG_* в r128_pkg.sv).
SEG = {
    "cs": 0,
    "ds": 1,
    "ss": 2,
    "es": 3,
}


@dataclass
class SourceLine:
    """Одна значимая строка исходного текста после удаления комментариев.

    address — адрес, на который эта строка будет собрана
              (вычисляется на первом проходе).
    """
    number: int
    text: str
    address: int


class AsmError(Exception):
    """Ошибка ассемблирования с привязкой к номеру строки исходника."""
    pass


# =============================================================================
# Вспомогательные функции разбора
# =============================================================================

COMMENT_RE = re.compile(r"(;|#).*$")
LABEL_RE = re.compile(r"^([A-Za-z_.$][A-Za-z0-9_.$]*):\s*(.*)$")
REG_RE = re.compile(r"^[rR](\d+)$")


def strip_comment(text: str) -> str:
    """Удаляет комментарий (; или #) и крайние пробелы."""
    return COMMENT_RE.sub("", text).strip()


def split_operands(text: str) -> list[str]:
    """
    Разделяет строку операндов по запятым с учётом вложенности скобок.

    Для текущей ISA достаточно простого разделения, однако поддержка
    глубины скобок позволяет в будущем писать выражения вида
    [base+offset] без изменения парсера.
    """
    out = []
    cur = []
    depth = 0

    for ch in text:
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1

        if ch == "," and depth == 0:
            item = "".join(cur).strip()
            if item:
                out.append(item)
            cur = []
        else:
            cur.append(ch)

    item = "".join(cur).strip()
    if item:
        out.append(item)

    return out


def parse_reg(token: str, line_no: int) -> int:
    """Разбирает регистр r0..r63. Возвращает номер регистра."""
    m = REG_RE.fullmatch(token.strip())
    if not m:
        raise AsmError(f"строка {line_no}: ожидался регистр r0..r63, получено '{token}'")

    n = int(m.group(1))
    if not 0 <= n <= 63:
        raise AsmError(f"строка {line_no}: регистр вне диапазона r0..r63: '{token}'")

    return n


def parse_segment(token: str, line_no: int) -> int:
    """
    Разбирает селектор сегмента.

    Допустимы символические имена cs/ds/ss/es либо число 0..3.
    """
    t = token.strip().lower()
    if t in SEG:
        return SEG[t]

    value = parse_number(token, {}, line_no)
    if not 0 <= value <= 3:
        raise AsmError(f"строка {line_no}: сегмент должен быть cs/ds/ss/es или 0..3")

    return value


def parse_number(token: str, symbols: dict[str, int], line_no: int) -> int:
    """
    Разбирает число или простой символ.

    Поддерживаемые формы:
      123, -123
      0x123, 0b1010, 0o77
      $FF          (альтернативный hex)
      label
      label+16
      label-16

    Выражения ограниченной сложности (только ± константа) достаточны
    для типичных нужд учебного ассемблера и не требуют полноценного
    парсера выражений.
    """
    t = token.strip()

    if t in symbols:
        return symbols[t]

    # Поддержка $FF как hex (стиль некоторых старых ассемблеров).
    if re.fullmatch(r"[-+]?[$][0-9A-Fa-f]+", t):
        sign = -1 if t.startswith("-") else 1
        s = t.lstrip("+-")[1:]
        return sign * int(s, 16)

    # Простые выражения symbol +/- constant.
    m = re.fullmatch(
        r"([A-Za-z_.$][A-Za-z0-9_.$]*)\s*([+-])\s*(0x[0-9A-Fa-f]+|0b[01]+|0o[0-7]+|\d+)",
        t,
    )
    if m:
        name, op, rhs = m.groups()
        if name not in symbols:
            raise AsmError(f"строка {line_no}: неизвестная метка '{name}'")
        value = int(rhs, 0)
        return symbols[name] + value if op == "+" else symbols[name] - value

    try:
        return int(t, 0)
    except ValueError:
        raise AsmError(f"строка {line_no}: неизвестное число/символ '{token}'")


def check_unsigned(value: int, bits: int, what: str, line_no: int) -> int:
    """Проверяет, что значение помещается в беззнаковое поле заданной ширины."""
    if not 0 <= value < (1 << bits):
        raise AsmError(
            f"строка {line_no}: {what}={value} не помещается в unsigned {bits} бит"
        )
    return value


def check_signed_or_unsigned(value: int, bits: int, what: str, line_no: int) -> int:
    """
    Разрешает либо обычное unsigned-значение, либо отрицательное signed.

    Отрицательное значение кодируется в дополнительном коде (two's complement)
    и возвращается уже как беззнаковое представление нужной ширины.
    Используется для поля imm (94 бита) и смещений переходов.
    """
    if value < 0:
        if value < -(1 << (bits - 1)):
            raise AsmError(
                f"строка {line_no}: {what}={value} не помещается в signed {bits} бит"
            )
        return value & ((1 << bits) - 1)

    if value >= (1 << bits):
        raise AsmError(
            f"строка {line_no}: {what}={value} не помещается в {bits} бит"
        )

    return value


def encode_instruction(opcode: int, rd: int, rs1: int, rs2: int, imm: int) -> bytes:
    """
    Собирает 128-битное слово инструкции и возвращает 16 байт little-endian.

    Порядок полей соответствует декодеру RTL:
        [127:112] opcode
        [111:106] rd
        [105:100] rs1
        [ 99: 94] rs2
        [ 93:  0] imm

    RTL и testbench ожидают little-endian представление слова в памяти:
    младший байт инструкции лежит по меньшему адресу.
    """
    word = (
        ((opcode & 0xFFFF) << 112)
        | ((rd & 0x3F) << 106)
        | ((rs1 & 0x3F) << 100)
        | ((rs2 & 0x3F) << 94)
        | (imm & ((1 << 94) - 1))
    )

    return word.to_bytes(16, byteorder="little", signed=False)


def instruction_size(mnemonic: str) -> int:
    """Все инструкции R128 имеют фиксированный размер 16 байт."""
    return 16


def resolve_label_or_number(token: str, symbols: dict[str, int], line_no: int) -> int:
    """Универсальный разбор операнда, который может быть меткой или числом."""
    return parse_number(token, symbols, line_no)


# =============================================================================
# Первый проход
#
# Строит таблицу символов (метки → адреса) и список значимых строк SourceLine.
# Директивы .org / .align меняют текущий PC; инструкции и данные увеличивают его.
# =============================================================================

def first_pass(lines: list[str]) -> tuple[dict[str, int], list[SourceLine], int]:
    symbols: dict[str, int] = {}
    source: list[SourceLine] = []
    pc = 0
    max_address = 0

    for line_no, raw in enumerate(lines, 1):
        text = strip_comment(raw)

        if not text:
            continue

        # Допускаются обе формы:
        #   label:
        #   label: add r1, r2, r3
        m = LABEL_RE.match(text)
        if m:
            label = m.group(1)

            if label in symbols:
                raise AsmError(f"строка {line_no}: повторная метка '{label}'")

            symbols[label] = pc
            text = m.group(2).strip()

            if not text:
                continue

        parts = text.split(None, 1)
        mnemonic = parts[0].lower()
        operands = parts[1].strip() if len(parts) > 1 else ""

        if mnemonic == ".org":
            if not operands:
                raise AsmError(f"строка {line_no}: .org требует адрес")
            pc = parse_number(operands, symbols, line_no)
            if pc < 0:
                raise AsmError(f"строка {line_no}: отрицательный .org")
            max_address = max(max_address, pc)
            source.append(SourceLine(line_no, text, pc))
            continue

        if mnemonic == ".align":
            if not operands:
                raise AsmError(f"строка {line_no}: .align требует степени/границу")
            alignment = parse_number(operands, symbols, line_no)
            if alignment <= 0 or (alignment & (alignment - 1)) != 0:
                raise AsmError(f"строка {line_no}: .align должен быть степенью 2")
            pc = (pc + alignment - 1) & ~(alignment - 1)
            max_address = max(max_address, pc)
            source.append(SourceLine(line_no, text, pc))
            continue

        if mnemonic == ".byte":
            values = split_operands(operands)
            if not values:
                raise AsmError(f"строка {line_no}: .byte требует данные")
            source.append(SourceLine(line_no, text, pc))
            pc += len(values)
            max_address = max(max_address, pc)
            continue

        if mnemonic == ".word":
            # 16-битные слова (little-endian)
            values = split_operands(operands)
            if not values:
                raise AsmError(f"строка {line_no}: .word требует данные")
            source.append(SourceLine(line_no, text, pc))
            pc += 2 * len(values)
            max_address = max(max_address, pc)
            continue

        if mnemonic == ".word128":
            # 128-битные слова — естественный размер данных процессора
            values = split_operands(operands)
            if not values:
                raise AsmError(f"строка {line_no}: .word128 требует данные")
            source.append(SourceLine(line_no, text, pc))
            pc += 16 * len(values)
            max_address = max(max_address, pc)
            continue

        if mnemonic == ".fill":
            values = split_operands(operands)
            if len(values) not in (1, 2):
                raise AsmError(f"строка {line_no}: .fill COUNT[,VALUE]")
            count = parse_number(values[0], symbols, line_no)
            if count < 0:
                raise AsmError(f"строка {line_no}: отрицательный .fill")
            source.append(SourceLine(line_no, text, pc))
            pc += count
            max_address = max(max_address, pc)
            continue

        if mnemonic not in OP:
            raise AsmError(f"строка {line_no}: неизвестная инструкция '{mnemonic}'")

        ops = split_operands(operands)
        validate_operand_count(mnemonic, ops, line_no)

        source.append(SourceLine(line_no, text, pc))
        pc += instruction_size(mnemonic)
        max_address = max(max_address, pc)

    return symbols, source, max_address


def validate_operand_count(mnemonic: str, ops: list[str], line_no: int) -> None:
    """
    Проверяет ожидаемое количество операндов для каждой инструкции.

    Диапазоны заданы жёстко в соответствии с форматом ISA.
    """
    counts = {
        "nop": (0, 0),
        "halt": (0, 0),

        "not": (2, 2),
        "mov": (2, 2),
        "in": (2, 2),
        "out": (2, 2),

        "add": (3, 3),
        "sub": (3, 3),
        "and": (3, 3),
        "or": (3, 3),
        "xor": (3, 3),
        "shl": (3, 3),
        "shr": (3, 3),
        "cmp": (2, 2),

        "load": (3, 3),
        "store": (3, 3),

        "jmp": (1, 1),
        "jz": (1, 1),
        "jnz": (1, 1),
        "jn": (1, 1),
        "jp": (1, 1),
        "jc": (1, 1),
        "jnc": (1, 1),
        "jv": (1, 1),
        "jnv": (1, 1),
        "je": (1, 1),
        "jne": (1, 1),
        "jugt": (1, 1),
        "jult": (1, 1),
        "juge": (1, 1),
        "jule": (1, 1),
        "jsgt": (1, 1),
        "jslt": (1, 1),
        "jsge": (1, 1),
        "jsle": (1, 1),

        "movs": (2, 2),
        "mfs": (2, 2),
        "loads": (4, 4),
        "stores": (4, 4),
        "jmpf": (2, 2),
    }

    lo, hi = counts[mnemonic]
    if not lo <= len(ops) <= hi:
        raise AsmError(
            f"строка {line_no}: {mnemonic} ожидает {lo}"
            + (f"..{hi}" if hi != lo else "")
            + f" операндов, получено {len(ops)}"
        )


# =============================================================================
# Второй проход — генерация машинного кода
#
# Для каждой инструкции формируется 128-битное слово согласно соглашениям RTL.
# Особое внимание уделяется относительным переходам и кодированию сегментов.
# =============================================================================

def assemble_instruction(
    mnemonic: str,
    ops: list[str],
    pc: int,
    symbols: dict[str, int],
    line_no: int,
) -> bytes:
    """
    Кодирует одну инструкцию.

    pc — адрес текущей инструкции (используется для вычисления
    относительного смещения переходов).
    """
    opcode = OP[mnemonic]

    # ----- Безоперандные -----
    if mnemonic in ("nop", "halt"):
        return encode_instruction(opcode, 0, 0, 0, 0)

    # ----- rd, rs1, rs2 -----
    if mnemonic in ("add", "sub", "and", "or", "xor", "shl", "shr"):
        rd = parse_reg(ops[0], line_no)
        rs1 = parse_reg(ops[1], line_no)
        rs2 = parse_reg(ops[2], line_no)
        return encode_instruction(opcode, rd, rs1, rs2, 0)

    # ----- rd, rs1 -----
    if mnemonic in ("not", "mov"):
        rd = parse_reg(ops[0], line_no)
        rs1 = parse_reg(ops[1], line_no)
        return encode_instruction(opcode, rd, rs1, 0, 0)

    # ----- cmp rs1, rs2 (результат не пишется, только флаги) -----
    if mnemonic == "cmp":
        rs1 = parse_reg(ops[0], line_no)
        rs2 = parse_reg(ops[1], line_no)
        return encode_instruction(opcode, 0, rs1, rs2, 0)

    # ----- load rd, base, imm -----
    # Адрес = base + sign_extend(imm). Сегмент по умолчанию — DS.
    if mnemonic == "load":
        rd = parse_reg(ops[0], line_no)
        rs1 = parse_reg(ops[1], line_no)
        imm = resolve_label_or_number(ops[2], symbols, line_no)
        imm = check_signed_or_unsigned(imm, 94, "imm", line_no)
        return encode_instruction(opcode, rd, rs1, 0, imm)

    # ----- store src, base, imm -----
    # RTL: rs1 = base, rs2 = данные для записи.
    # Порядок операндов в ассемблере: store <src>, <base>, <imm>
    if mnemonic == "store":
        src = parse_reg(ops[0], line_no)
        base = parse_reg(ops[1], line_no)
        imm = resolve_label_or_number(ops[2], symbols, line_no)
        imm = check_signed_or_unsigned(imm, 94, "imm", line_no)
        return encode_instruction(opcode, 0, base, src, imm)

    # ----- Относительные переходы -----
    # RTL вычисляет: target = PC + sign_extend(imm)
    # Поэтому ассемблер должен положить в imm значение (label − PC).
    # Если операнд — чистое число, оно трактуется уже как готовое смещение.
    branches = {
        "jmp", "jz", "jnz", "jn", "jp", "jc", "jnc",
        "jv", "jnv", "je", "jne", "jugt", "jult", "juge",
        "jule", "jsgt", "jslt", "jsge", "jsle",
    }

    if mnemonic in branches:
        target = resolve_label_or_number(ops[0], symbols, line_no)

        if ops[0].strip() in symbols or re.match(
            r"^[A-Za-z_.$][A-Za-z0-9_.$]*(?:\s*[+-]\s*.*)?$", ops[0].strip()
        ):
            displacement = target - pc
        else:
            # Число трактуем как уже готовое относительное смещение.
            displacement = target

        imm = check_signed_or_unsigned(
            displacement, 94, "branch displacement", line_no
        )
        return encode_instruction(opcode, 0, 0, 0, imm)

    # ----- MOVS segment, register -----
    # RTL: id_ex_rd[1:0] выбирает сегмент, rs1 — источник.
    if mnemonic == "movs":
        seg = parse_segment(ops[0], line_no)
        src = parse_reg(ops[1], line_no)
        return encode_instruction(opcode, seg, src, 0, 0)

    # ----- MFS register, segment -----
    # RTL: rs1[1:0] выбирает сегмент.
    if mnemonic == "mfs":
        rd = parse_reg(ops[0], line_no)
        seg = parse_segment(ops[1], line_no)
        return encode_instruction(opcode, rd, seg, 0, 0)

    # ----- LOADS rd, base, segment, imm -----
    # RTL: rs2[1:0] = segment selector.
    if mnemonic == "loads":
        rd = parse_reg(ops[0], line_no)
        base = parse_reg(ops[1], line_no)
        seg = parse_segment(ops[2], line_no)
        imm = resolve_label_or_number(ops[3], symbols, line_no)
        imm = check_signed_or_unsigned(imm, 94, "imm", line_no)
        return encode_instruction(opcode, rd, base, seg, imm)

    # ----- STORES segment, base, src, imm -----
    # RTL: rd[1:0] = segment selector, rs1 = base, rs2 = data.
    if mnemonic == "stores":
        seg = parse_segment(ops[0], line_no)
        base = parse_reg(ops[1], line_no)
        src = parse_reg(ops[2], line_no)
        imm = resolve_label_or_number(ops[3], symbols, line_no)
        imm = check_signed_or_unsigned(imm, 94, "imm", line_no)
        return encode_instruction(opcode, seg, base, src, imm)

    # ----- JMPF segment, pc -----
    # RTL:
    #   imm[15:0]  = CS
    #   imm[31:16] = новый PC
    # PC в архитектуре ограничен 16 битами.
    if mnemonic == "jmpf":
        cs = parse_segment(ops[0], line_no)
        target_pc = resolve_label_or_number(ops[1], symbols, line_no)

        if not 0 <= target_pc <= 0xFFFF:
            raise AsmError(
                f"строка {line_no}: JMPF PC={target_pc:#x} вне диапазона 16 бит"
            )

        imm = cs | (target_pc << 16)
        return encode_instruction(opcode, 0, 0, 0, imm)

    # ----- IN rd, port -----
    # Порт кодируется в младших 8 битах imm.
    if mnemonic == "in":
        rd = parse_reg(ops[0], line_no)
        port = resolve_label_or_number(ops[1], symbols, line_no)
        port = check_unsigned(port, 8, "I/O port", line_no)
        return encode_instruction(opcode, rd, 0, 0, port)

    # ----- OUT rs, port -----
    if mnemonic == "out":
        src = parse_reg(ops[0], line_no)
        port = resolve_label_or_number(ops[1], symbols, line_no)
        port = check_unsigned(port, 8, "I/O port", line_no)
        return encode_instruction(opcode, 0, src, 0, port)

    raise AsmError(f"строка {line_no}: внутренняя ошибка: не обработан '{mnemonic}'")


def assemble(
    text: str,
) -> tuple[bytes, dict[str, int], list[tuple[int, int, str, bytes]]]:
    """
    Полный цикл ассемблирования.

    Возвращает:
      image   — байтовый образ памяти;
      symbols — таблица меток;
      listing — список (address, line_no, text, data) для генерации листинга.
    """
    lines = text.splitlines()
    symbols, source, max_address = first_pass(lines)

    image = bytearray(max_address)
    listing = []

    for src in source:
        line_no = src.number
        text_line = src.text

        parts = text_line.split(None, 1)
        mnemonic = parts[0].lower()
        operands = parts[1].strip() if len(parts) > 1 else ""

        if mnemonic == ".org":
            # Адрес уже вычислен на первом проходе.
            continue

        if mnemonic == ".align":
            continue

        if mnemonic == ".byte":
            values = split_operands(operands)
            data = bytes(
                check_unsigned(
                    parse_number(v, symbols, line_no), 8, ".byte", line_no
                )
                for v in values
            )
            end = src.address + len(data)
            if end > len(image):
                image.extend(b"\x00" * (end - len(image)))
            image[src.address:end] = data
            listing.append((src.address, line_no, text_line, data))
            continue

        if mnemonic == ".word":
            values = split_operands(operands)
            data = b"".join(
                check_unsigned(
                    parse_number(v, symbols, line_no), 16, ".word", line_no
                ).to_bytes(2, "little")
                for v in values
            )
            end = src.address + len(data)
            if end > len(image):
                image.extend(b"\x00" * (end - len(image)))
            image[src.address:end] = data
            listing.append((src.address, line_no, text_line, data))
            continue

        if mnemonic == ".word128":
            values = split_operands(operands)
            data_parts = []
            for v in values:
                n = parse_number(v, symbols, line_no)
                n = check_unsigned(n, 128, ".word128", line_no)
                data_parts.append(n.to_bytes(16, "little"))
            data = b"".join(data_parts)
            end = src.address + len(data)
            if end > len(image):
                image.extend(b"\x00" * (end - len(image)))
            image[src.address:end] = data
            listing.append((src.address, line_no, text_line, data))
            continue

        if mnemonic == ".fill":
            values = split_operands(operands)
            count = parse_number(values[0], symbols, line_no)
            value = parse_number(values[1], symbols, line_no) if len(values) == 2 else 0
            value = check_unsigned(value, 8, ".fill value", line_no)

            data = bytes([value]) * count
            end = src.address + len(data)
            if end > len(image):
                image.extend(b"\x00" * (end - len(image)))
            image[src.address:end] = data
            listing.append((src.address, line_no, text_line, data))
            continue

        ops = split_operands(operands)
        data = assemble_instruction(mnemonic, ops, src.address, symbols, line_no)

        end = src.address + 16
        if end > len(image):
            image.extend(b"\x00" * (end - len(image)))
        image[src.address:end] = data
        listing.append((src.address, line_no, text_line, data))

    return bytes(image), symbols, listing


# =============================================================================
# Вывод результатов
# =============================================================================

def make_hex_dump(data: bytes, width: int = 16) -> str:
    """Классический hex-dump с ASCII-колонкой."""
    lines = []
    for address in range(0, len(data), width):
        chunk = data[address:address + width]
        hex_part = " ".join(f"{b:02X}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{address:08X}  {hex_part:<{width*3-1}}  |{ascii_part}|")
    return "\n".join(lines)


def make_listing(listing: list[tuple[int, int, str, bytes]]) -> str:
    """
    Формирует листинг ассемблера.

    Формат строки:
        ADDR  HEX-BYTES...  LINE  SOURCE
    """
    out = []
    for address, line_no, text, data in listing:
        hex_part = " ".join(f"{b:02X}" for b in data)
        out.append(f"{address:06X}  {hex_part:<47}  {line_no:4d}  {text}")
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="R128 assembler — one-file Python assembler for the R128 CPU"
    )
    parser.add_argument("source", help="ASM source file")
    parser.add_argument(
        "-o", "--output",
        help="output binary file; default: source basename.bin",
    )
    parser.add_argument(
        "--listing",
        help="write assembler listing",
    )
    parser.add_argument(
        "--hex",
        action="store_true",
        help="print assembled image as hex dump",
    )
    parser.add_argument(
        "--symbols",
        action="store_true",
        help="print symbol table",
    )
    parser.add_argument(
        "--no-trim",
        action="store_true",
        help="do not trim trailing zero bytes from the image",
    )

    args = parser.parse_args()

    source_path = Path(args.source)

    try:
        text = source_path.read_text(encoding="utf-8")
    except OSError as exc:
        print(f"ERROR: не удалось прочитать {source_path}: {exc}", file=sys.stderr)
        return 1

    try:
        image, symbols, listing = assemble(text)
    except AsmError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    if not args.no_trim:
        image = image.rstrip(b"\x00")

    output_path = (
        Path(args.output)
        if args.output
        else source_path.with_suffix(".bin")
    )

    try:
        output_path.write_bytes(image)
    except OSError as exc:
        print(f"ERROR: не удалось записать {output_path}: {exc}", file=sys.stderr)
        return 1

    print("R128 ASSEMBLER")
    print(f"  source:  {source_path}")
    print(f"  output:  {output_path}")
    print(f"  size:    {len(image)} bytes")
    print(f"  words:   {(len(image) + 15) // 16} x 128-bit")

    if symbols:
        print("\nSYMBOLS:")
        for name, addr in sorted(symbols.items(), key=lambda x: x[1]):
            print(f"  {name:<24} 0x{addr:04X}")

    if args.listing:
        try:
            Path(args.listing).write_text(
                make_listing(listing) + "\n",
                encoding="utf-8",
            )
        except OSError as exc:
            print(f"ERROR: не удалось записать listing: {exc}", file=sys.stderr)
            return 1
        print(f"  listing: {args.listing}")

    if args.hex:
        print("\nHEX DUMP:")
        print(make_hex_dump(image))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
