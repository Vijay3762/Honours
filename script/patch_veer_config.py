#!/usr/bin/env python3
"""
patch_veer_config.py -- patch the VeeR EL2 configuration.

The generated configuration (rtl/cpu/veer_el2/config/el2_param.vh) holds the
core parameters as one 2294 bit packed literal, which is not practical to
edit by hand.  This script decodes the literal using the packed struct
definition in el2_pdef.vh, applies the requested changes and writes the
literal back.

The SoC uses the core as a pure AXI master, so the local memories are
switched off:

    ICCM_ENABLE   = 0   instruction cache memory  (no external SRAM needed)
    ICCM_ICACHE  = 0
    DCCM_ENABLE   = 0   data cache memory         (no external SRAM needed)
    ICACHE_ENABLE = 0   instruction cache         (fetch straight from AXI)

With the instruction cache disabled the IFU fetches every 64-bit beat
directly from the AXI bus, so the SoC only needs the axi_mem models.

Usage:
    python3 script/patch_veer_config.py            # apply the default patch
    python3 script/patch_veer_config.py --show     # print the decoded config
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.normpath(os.path.join(HERE, ".."))
CONFIG = os.path.join(PROJ, "rtl", "cpu", "veer_el2", "config")
PDEF = os.path.join(CONFIG, "el2_pdef.vh")
PARAM = os.path.join(CONFIG, "el2_param.vh")
SIMPLE = os.path.join(CONFIG, "el2_param_simple.vh")

FIELD_RE = re.compile(r"^\s*logic\s*(?:\[\s*(\d+)\s*:\s*(\d+)\s*\])?\s*([A-Za-z_][A-Za-z0-9_]*)\s*;")
# The literal is the last item of a parameter port list, so it has no
# terminating semicolon.
LITERAL_RE = re.compile(r"parameter\s+el2_param_t\s+pt\s*=\s*(\d+)'h([0-9a-fA-F]+)")

# name -> value
#
# The SoC uses the core with the ICCM as its local instruction memory:
#   ICCM_ENABLE = 1, ICCM_ONLY = 1  -> the instructions come from the ICCM
#   ICACHE_ENABLE = 0                -> no instruction cache, no external
#                                      I$ SRAM model needed
#   DCCM_ENABLE = 0                  -> no data cache memory
#
# These are the values the VeeR configuration generator derives for the
# iccache_enable=0 / iccm_enable=1 / dccm_enable=0 combination.
DEFAULT_PATCH = {
    "ICCM_ENABLE": 1,
    "ICCM_ICACHE": 0,
    "ICCM_ONLY": 1,
    "DCCM_ENABLE": 0,
    "ICACHE_ENABLE": 0,
    "ICACHE_ONLY": 0,
    "NO_ICCM_NO_ICACHE": 0,
    # the fast interrupt redirect needs the DCCM
    "FAST_INTERRUPT_REDIRECT": 0,
}


def parse_fields(path):
    """Return [(name, width)] in declaration order."""
    fields = []
    inside = False
    with open(path) as f:
        for line in f:
            if not inside:
                if "typedef struct packed" in line:
                    inside = True
                continue
            if "} el2_param_t;" in line:
                break
            m = FIELD_RE.match(line)
            if m:
                hi, lo, name = m.groups()
                width = (int(hi) - int(lo) + 1) if hi is not None else 1
                fields.append((name, width))
    return fields


def build_offsets(fields):
    """Bit offset (from the LSB) of every field of the packed struct."""
    offsets = {}
    pos = 0
    for name, width in reversed(fields):
        offsets[name] = pos
        pos += width
    return offsets, pos


def decode(value, fields, offsets):
    out = {}
    for name, width in fields:
        out[name] = (value >> offsets[name]) & ((1 << width) - 1)
    return out


def encode(values, fields, offsets):
    value = 0
    for name, width in fields:
        value |= (values[name] & ((1 << width) - 1)) << offsets[name]
    return value


def read_literal(path):
    with open(path) as f:
        text = f.read()
    m = LITERAL_RE.search(text)
    if not m:
        raise SystemExit("ERROR: no el2_param_t literal found in %s" % path)
    width = int(m.group(1))
    value = int(m.group(2), 16)
    return width, value, text, m


def read_simple(path):
    """The human readable version of the same configuration (for checking)."""
    values = {}
    if not os.path.exists(path):
        return values
    with open(path) as f:
        for line in f:
            m = re.match(r"\s*\.([A-Za-z_][A-Za-z0-9_]*)\((\d+)'d(\d+)\)", line)
            if m:
                values[m.group(1)] = int(m.group(3))
    return values


def main():
    fields = parse_fields(PDEF)
    if not fields:
        raise SystemExit("ERROR: could not parse the struct in %s" % PDEF)
    offsets, total = build_offsets(fields)

    width, value, text, m = read_literal(PARAM)
    if width != total:
        raise SystemExit("ERROR: literal is %d bits, struct is %d bits"
                         % (width, total))

    values = decode(value, fields, offsets)

    if "--show" in sys.argv:
        for name, w in fields:
            print("%-32s %4d bit(s)  = %d" % (name, w, values[name]))
        return 0

    # ---- sanity check against the readable reference ------------------------
    ref = read_simple(SIMPLE)
    mismatches = []
    for name, refval in ref.items():
        if name in values and values[name] != refval:
            mismatches.append("%s: packed=%d reference=%d"
                              % (name, values[name], refval))
    if mismatches:
        print("WARNING: differences between the packed literal and "
              "el2_param_simple.vh:")
        for msg in mismatches:
            print("    " + msg)
        print("")

    # ---- apply the patch ----------------------------------------------------
    changed = []
    for name, newval in DEFAULT_PATCH.items():
        if name not in values:
            raise SystemExit("ERROR: unknown parameter %s" % name)
        if values[name] != newval:
            changed.append("%s: %d -> %d" % (name, values[name], newval))
            values[name] = newval

    if not changed:
        print("configuration already patched, nothing to do")
        return 0

    newvalue = encode(values, fields, offsets)
    newlit = "parameter el2_param_t pt = %d'h%0*X" % (total, (total + 3) // 4,
                                                      newvalue)
    newtext = text[:m.start()] + newlit + text[m.end():]
    with open(PARAM, "w") as f:
        f.write(newtext)

    print("patched %s" % PARAM)
    for msg in changed:
        print("    " + msg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
