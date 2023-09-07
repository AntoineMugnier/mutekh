import click
import math

acc_unit = {
    "ppm": 1e3,
    "": 1e3,
    "ppb": 1,
    "%": 1e7,
}

freq_unit = {
    "MHz": 1e6,
    "kHz": 1e3,
    "GHz": 1e9,
    "Hz": 1,
}

def acc_pretty(ppb):
    if ppb is None:
        return "<Inval>"
    for (unit, exp) in sorted(acc_unit.items(), key = lambda x:x[1], reverse = True):
        if ppb >= exp:
            value = ppb/exp
            if value < 10:
                v = f"{value:1.2f}"
            elif value < 100:
                v = f"{value:2.1f}"
            else:
                v = f"{int(value):d}"
            return f"{v} {unit}"
    return f"{ppb:0.2} ppb"

def acc_parse(s):
    for (unit, mult) in sorted(acc_unit.items(), key = lambda x:x[0], reverse = True):
        if s.endswith(unit):
            return float(s[:-len(unit)]) * mult
    raise ValueError(f"Cannot parse accuracy string {str}")

def freq_pretty(hz):
    for (unit, exp) in sorted(freq_unit.items(), key = lambda x:-x[1]):
        if hz >= exp:
            return f"{hz/exp:2.1f} {unit}"

def freq_parse(s):
    for (unit, mult) in sorted(freq_unit.items(), key = lambda x:-x[1]):
        if s.endswith(unit) and unit:
            return float(s[:-len(unit)]) * mult
    raise ValueError(f"Cannot parse frequency string {str}")

def acc_calc(m, e):
    if m == 0 and e == 0:
        return None
    return float(8 | m) * 2.0 ** (e - 4)

def acc_extract(value, mode = "nearest"):
    msb = int(math.log2(value))

    m = value * 2.0 ** (-msb+3)
    if mode == "above":
        m = int(math.ceil(m))
    elif mode == "nearest":
        m = int(m + .5)
    else:
        m = int(m)

    assert (m & ~7) == 0x8

    acc_m = m & 7
    acc_e = msb + 1

    assert (acc_e) <= 0x1f

@click.group()
def precision():
    pass

def acc_pp(acc_m, acc_e, frequency = None):
    ppb = acc_calc(acc_m, acc_e)

    print(f"acc_m: {acc_m}")
    print(f"acc_e: {acc_e}")
    print(f"{acc_m}, {acc_e}")

    acc_str = acc_pretty(ppb)

    print(f"Accuracy value: {acc_str}")
    
    if frequency is not None:
        frequency = freq_parse(frequency)
        
        fmin = frequency * (1 - ppb * 1e-9)
        fmax = frequency * (1 + ppb * 1e-9)
        print(f"Frequency range: {freq_pretty(fmin)} - {freq_pretty(fmax)}")

        print(f"DEV_FREQ({int(frequency)}, 1, {acc_m}, {acc_e}) // {freq_pretty(frequency)} +/- {acc_str}: {freq_pretty(fmin)} - {freq_pretty(fmax)}")

@precision.command()
@click.argument("precision", type = str)
@click.option("--frequency", "-f", type = str, default = None)
@click.option("--mode", "-m", type = str, default = "nearest", help = "above|nearest|below")
def encode(precision, mode, frequency):
    value = acc_parse(precision)
    acc_m, acc_e = acc_extract(value, mode)
    acc_pp(acc_m, acc_e, frequency)

@precision.command()
@click.argument("acc_m", type = int)
@click.argument("acc_e", type = int)
@click.option("--frequency", "-f", type = str, default = None)
def decode(acc_m, acc_e, frequency):
    assert 0 <= acc_m < 8
    assert 1 <= acc_e < 31

    acc_pp(acc_m, acc_e, frequency)

@precision.command()
def table():
    print("e \\ m", end = " ")
    for acc_m in range(8):
        print(f"{acc_m}".rjust(9), end = " ")
    print()
    for acc_e in range(32):
        print(f"{acc_e:5d}", end = " ")
        for acc_m in range(8):
            print(acc_pretty(acc_calc(acc_m, acc_e)).rjust(9), end = " ")
        print()
            
    
if __name__ == "__main__":
    precision()
