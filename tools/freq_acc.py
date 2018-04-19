#!/usr/bin/env python3

import argparse
import math

def acc_fp_to_ppb(m, e):
    m = int(m)
    assert m < 8
    m |= 8
    if e >= 4:
        return m << (e - 4)
    return m >> (4 - e)

def acc_fp_from_ppb(ppb):
    if ppb < 1:
        ppb = 1
    ppb = int(ppb)

    m = ppb * 16.
    e = -4

    while m >= 15:
        m /= 2
        e += 1

    e += 4

    m = int(math.ceil(m))

    assert m & 0x8
    m &= 0x7

    return m, e

def main():
    parser = argparse.ArgumentParser(description = "Clock accuracy constant calculator")
    parser.add_argument("precision", type = str, help = "Clock precision")

    args = parser.parse_args()

    prec_str = args.precision.lower()

    if prec_str.endswith("%"):
        target_ppb = float(prec_str[:-1]) * 1e7
    elif prec_str.endswith("ppm"):
        target_ppb = float(prec_str[:-3]) * 1e3
    elif prec_str.endswith("ppb"):
        target_ppb = float(prec_str[:-3])
    else:
        raise ValueError("Precision should end with 'ppb', 'ppm' or '%'")

    m, e = acc_fp_from_ppb(target_ppb)
    ppb = acc_fp_to_ppb(m, e)

    print("Target: %d ppb" % target_ppb)
    print("Effective: %d ppb" % ppb)
    print("Parameters (m, e): %d, %d" % (m, e))

if __name__ == "__main__":
    main()
