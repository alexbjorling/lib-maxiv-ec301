import time
import select
import struct
from collections import deque

BUF_SIZE = 4096
TIMEOUT = .1

def _clear_incoming_buffer(s):
    """ 
    Clears the incoming buffer on socket s.

    Returns: remaining data.
    """
    buf = ''
    while True:
        ready = select.select([s], [], [], TIMEOUT)
        if ready[0]:
            new = s.recv(BUF_SIZE)
            buf += new
            print 'clearing %d bytes' % len(new)
        else:
            break
    return buf


class StreamReceiver(object):
    """
    Class responsible for catching binary data streams from the EC301.

    Sort of works. For averaging=1 it seems we miss some data.

    We should probably read and parse at the same time so we can monitor scanning status etc.
    """
    def __init__(self, dev):
        self.dev = dev
        self.buf = ''

    def start(self):
        print 'clearing %d bytes first' % len(_clear_incoming_buffer(self.dev.socket))
        self.dev._query('getbda 1')
        t0 = time.time()
        self.buf = ''
        while time.time() < t0 + 1.07321876:
            ready = select.select([self.dev.socket], [], [], TIMEOUT)
            if ready[0]:
                self.buf += self.dev.socket.recv(BUF_SIZE)
        self.dev._query('getbda 0')
        self.buf += _clear_incoming_buffer(self.dev.socket)
        print "received total of %d bytes" % len(self.buf)

    def parse(self):
        offset = 0
        E, I, aux, running, raw = [], [], [], [], []
        while offset < len(self.buf):
            # starting on a new packet
            pkt_counter, payload = struct.unpack('xxHIxxxxxxxx', self.buf[offset:16+offset])
            n_frames = payload / 16
            # fast state bitfields, ADC, I data, E data
            frame_fmt = 'ifff'
            # flags for ramp, pulse, and arbitrary waveform
            scan_running_mask = 1 << 22 | 1 << 19 | 1 << 11
            print pkt_counter, n_frames, offset, len(self.buf)
            for i in range(n_frames):
                fast, aux_, I_, E_ = struct.unpack(frame_fmt, self.buf[16+16*i+offset:16+16*(i+1)+offset])
                running_ = bool(fast & scan_running_mask)
                E.append(E_)
                I.append(I_)
                aux.append(aux_)
                running.append(running_)
                raw.append(fast)
            offset += 16 + 16 * n_frames + 24

        print pkt_counter, n_frames, offset, len(self.buf)

        return E, I, aux, running, raw

from EC301 import EC301
e=EC301(debug=False)
s = StreamReceiver(e)
#s.start()
#E, I, aux, running, raw = s.parse()

