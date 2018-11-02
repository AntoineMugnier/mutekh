#!/usr/bin/env python3

import click
import zlib
import struct

@click.command()
@click.argument("binary", type=click.File('rb'))
@click.argument("dfu", type=click.File('wb'))
@click.option('--vid', help='USB VendorID', default = "ffff")
@click.option('--pid', help='USB ProductID', default = "ffff")
@click.option('--release', help='USB DeviceVersion', default = "ffff")
def dfu_gen(binary, dfu, vid, pid, release):
    """Generate DFU file from binary"""
    vid = int(vid, 16)
    pid = int(pid, 16)
    release = int(release, 16)

    blob = binary.read()
    suffix = struct.pack("<HHHH3sB", release, pid, vid, 0x100, b'UFD', 16)
    crc = zlib.crc32(blob + suffix) ^ 0xffffffff
    dfu.write(blob)
    dfu.write(suffix)
    dfu.write(crc.to_bytes(4, "little"))

if __name__ == '__main__':
    dfu_gen()
