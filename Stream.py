import time
import select
import struct
import numpy as np
from collections import deque
import threading

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
        else:
            break
    return buf


class Stream(object):
    """
    Class responsible for capturing and parsing binary data
    streams from the EC301.

    One-time objects, create a new instance for every scan.

    Data is accessed with the public attributes, the member 'done'
    is True when data collection is finished.
    """
    def __init__(self, dev=None, complete_scan=True, max_data_points=None, debug=False):
        """
        dev (EC301 object):     EC301 device instance to speak to.
        complete_scan (bool):   Close the stream when a running scan finishes.
        max_data_points (int):  Close the stream if this many data points are read.
        """
        # Internal attributes
        self.dev = dev
        self.complete_scan = bool(complete_scan)
        self.max_data_points = int(max_data_points) if max_data_points is not None else None
        self.buf = ''
        self.do_debug = debug

        # Public attributes
        self.E = []
        self.I = []
        self.aux = []
        self.running = []
        self.raw = []
        self.done = False
        self.cancel = False

    def debug(self, msg):
        if self.do_debug:
            print msg

    def start(self):
        """
        Start stream and record/parse data in a separate thread.
        """
        if self.done:
            raise Exception('This stream has already been run.')
        if self.dev is None:
            raise Exception('This stream is probably a dummy as it\'s not associated with an EC301 instance.')
        t = threading.Thread(target=self._start)
        t.start()

    def _start(self):
        """
        Actual worker function.
        """
        self.debug('clearing %d bytes first' % len(_clear_incoming_buffer(self.dev.socket)))
        self.dev._query('getbda 1')
        t0 = time.time()
        self.buf = ''
        for i_ in self.parser():
            if self.cancel:
                self.cancel = False
                break
            ready = select.select([self.dev.socket], [], [], TIMEOUT)
            if ready[0]:
                self.buf += self.dev.socket.recv(BUF_SIZE)
                self.debug("read data, total %d" % len(self.buf))
        self.dev._query('getbda 0')
        self.debug('cleared %d bytes afterwards' % len(_clear_incoming_buffer(self.dev.socket)))
        self.debug("received total of %d bytes" % len(self.buf))
        self.done = True

    def parser(self):
        """
        Generator that parses the incoming buffer as far as it can and 
        attaches extracted data as object attributes.
        """
        offset = 0
        frame_fmt = 'ifff'                              # data frame field format
        scan_running_mask = 1 << 22 | 1 << 19 | 1 << 11 # within the fast bit field, these flags mark scan running

        while True:
            # do we have enough data to read a header?
            while len(self.buf) < offset + 16:
                self.debug("yielding, no complete header, len(self.buf) = %d" % len(self.buf))
                yield
            self.debug("*reading a header")
            pkt_counter, payload = struct.unpack('xxHIxxxxxxxx', self.buf[offset:16+offset])
            n_frames = payload / 16
            # do we have enough data to read an entire packet?
            while len(self.buf) < offset + 16 + 16 * n_frames + 24:
                self.debug("yielding, no complete packet, len(self.buf) = %d" % len(self.buf))
                yield
            self.debug("*reading complete packet %d with %d frames"%(pkt_counter, n_frames))
            for i in range(n_frames):
                fast, aux_, I_, E_ = struct.unpack(frame_fmt, self.buf[16+16*i+offset:16+16*(i+1)+offset])
                running_ = bool(fast & scan_running_mask)
                self.E.append(E_)
                self.I.append(I_)
                self.aux.append(aux_)
                self.running.append(running_)
                self.raw.append(fast)
                self.debug('packet %d frame %d, scanning? %s' % (pkt_counter, i, running_))
                # end conditions
                if self.max_data_points and (len(self.E) >= self.max_data_points):
                    return
                if self.complete_scan and len(self.running) > 1 and self.running[-2] and not self.running[-1]:
                    return
            offset += 16 + 16 * n_frames + 24

if __name__ == '__main__':
    from EC301 import EC301
    e=EC301(debug=False)
#    s = Stream(e, max_data_points=1e3)
#    s.start()
#    self.debug(s.done
#    self.debug(s.E[:20]
#    self.debug("got %d data points" % len(s.E)

