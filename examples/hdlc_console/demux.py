#!/usr/bin/python3

import serial
import serial_asyncio
import asyncio
import os

class HdlcMux(asyncio.Protocol):
    ST_UNSYNC = 0
    ST_DATA = 1

    def __init__(self, frame_size = 48):
        self.registry = {}
        self.state = self.ST_UNSYNC
        self.buffer = b""

    def register(self, id, term):
        if not id in self.registry:
            self.registry[id] = set()
        self.registry[id].add(term)

    def unregister(self, id, term):
        if id in self.registry:
            self.registry[id].remove(term)

    def connection_made(self, transport):
        self.transport = transport

    def data_received(self, data):
        #print(f"Mux < {data.hex()}")
        self.buffer += data
        self.process()

    def process(self):
        while True:
#            print(f"In state {self.state}, Waiting data: {self.buffer.hex()}")
            if self.state == self.ST_UNSYNC:
                try:
                    i = self.buffer.index(b"\x7e")
                except:
                    self.buffer = b""
                    return
                self.buffer = self.buffer[i:].lstrip(b"\x7e")
                if not self.buffer:
                    self.buffer = b"\x7e"
                    return
                self.state = self.ST_DATA
                continue

            elif self.state == self.ST_DATA:
                try:
                    i = self.buffer.index(b"\x7e")
                except:
                    return
                pkt = self.buffer[:i]
                self.buffer = self.buffer[i+1:]
                self.state = self.ST_DATA

                if pkt:
                    print(f">> {pkt.hex()}")
                    self.handle(pkt)

    @classmethod
    def crc(self, data, init = 0):
        import crcmod
        c = crcmod.Crc(0x11021, initCrc = init ^ 0xffff)
        c.update(data)
        return (0xffff ^ int.from_bytes(c.digest(), "big")).to_bytes(2, "little")

    @classmethod
    def escape(self, data):
        r = []
        for i in data:
            if i in [0x7d, 0x7e, 0x11, 0x13, 0x91, 0x93, 0x03]:
                r.append(0x7d)
                r.append(i ^ 0x20)
            else:
                r.append(i)
        return bytes(r)

    @classmethod
    def unescape(self, data):
        r = []
        escaped = False
        for i in data:
            if escaped:
                escaped = False
                r.append(i ^ 0x20)
            elif i == 0x7d:
                escaped = True
            else:
                r.append(i)
        return bytes(r)

    def handle(self, frame):
        frame = self.unescape(frame)
        if self.crc(frame[:-2]) !=  frame[-2:]:
            print(f"** BAD: {frame.hex()} {self.crc(frame[:-2]).hex()} {self.crc(frame).hex()}")
            return
        addr = frame[0]
        cmd = frame[1]
        data = frame[2:-2]
        target = self.registry.get(addr, [])
        #print(f"PKT < {addr:02x}{cmd:02x} {str(data)}")
        for t in target:
            t.receive(cmd, data)

    def connection_lost(self, exc):
        print('port closed')
        asyncio.get_event_loop().stop()

    def write_chunk(self, addr, cmd, data):
        header = bytes([addr, cmd])
        crc = self.crc(header + data)
        frame = b'\x7e' + self.escape(header + data + crc) + b'\x7e'
        print(f"<< {frame.hex()}")
        self.transport.write(frame)

class MuxedChar(asyncio.Protocol):
    def __init__(self, mux, id):
        self.mux = mux
        self.id = id
        mux.register(id, self)
        self.transport = None

    def connection_made(self, transport):
        self.transport = transport

    def send(self, data):
        self.mux.write_chunk(self.id, 3, data)

    def data_received(self, data):
        #print(f"CH{self.id} <      {data.hex()}")
        self.send(data)

    def receive(self, cmd, data):
        #print(f"CH{self.id} >      {data.hex()}")
        self.transport.write(data)

    def connection_lost(self, exc):
        self.mux.unregister(self.id, self)

class MuxedSocket(MuxedChar):
    @classmethod
    def listen(cls, mux, id):
        port = 4700 + id
        factory = lambda: cls(mux, id)
        loop = asyncio.get_event_loop()
        coro = loop.create_server(factory, '127.0.0.1', port)
        loop.run_until_complete(coro)
        print("Registered mux channel #{} on port {}".format(id, port))

class PtsSocket:
    PTMX = "/dev/ptmx"
    family = None
    libc = None

    @classmethod
    def _init(cls):
        if cls.libc:
            return

        from ctypes import c_int, c_char_p, cdll

        cls.libc = cdll.LoadLibrary("libc.so.6")
        cls.libc.ptsname.args = [c_int]
        cls.libc.ptsname.restype = c_char_p

    def __init__(self):
        self._init()

        self.ptmx = os.open(self.PTMX, os.O_RDWR|os.O_NOCTTY)
        self.libc.grantpt(self.ptmx)
        self.libc.unlockpt(self.ptmx)

    def getsockname(self):
        return self.PTMX

    def getpeername(self):
        return self.libc.ptsname(self.ptmx)

    def fileno(self):
        return self.ptmx

    def close(self):
        os.close(self.ptmx)

    def read(self, n):
        try:
            return os.read(self.ptmx, n)
        except OSError as e:
            print(str(self.getpeername(), "ascii"), "closed")
            return None
    recv = read

    def write(self, b):
        return os.write(self.ptmx, b)
    send = write

class MuxedPts(MuxedChar):
    @classmethod
    def listen(cls, mux, id):
        loop = asyncio.get_event_loop()
        pts = PtsSocket()
        loop._make_socket_transport(pts, cls(mux, id))
        print("Registered mux channel #{} on {}".format(id, pts.getpeername()))

def main():
    modes = dict(pts = MuxedPts, tcp = MuxedSocket)

    import argparse
    parser = argparse.ArgumentParser("Charmux to TCP socket demuxer")
    parser.add_argument("port", help = "Serial port", type = str)
    parser.add_argument("-r", "--rate", help = "Serial port baudrate", default = 115200, type = int)
    parser.add_argument("-c", "--count", help = "Mux count", default = 3, type = int)
    parser.add_argument("-s", "--socket", help = "Create TCP sockets (default)", action = "store_const", dest = "mode", const = "tcp", default = "tcp")
    parser.add_argument("-p", "--pts", help = "Create PTS", action = "store_const", dest = "mode", const = "pts")

    args = parser.parse_args()

    target_class = modes[args.mode]
    mux = HdlcMux()
    for i in range(args.count):
        target_class.listen(mux, i)

    loop = asyncio.get_event_loop()
    coro = serial_asyncio.create_serial_connection(loop, lambda : mux, args.port, baudrate = args.rate)
    loop.run_until_complete(coro)
    try:
        loop.run_forever()
    except KeyboardInterrupt as e:
        loop.close()
    except:
        raise

if __name__ == "__main__":
    main()
