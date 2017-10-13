import socket
import select
import time
import numpy as np

# hardware notes: 
#  you can open several sockets at the same time and send commands independently, tested this.
#  if you send many commands in rapid succession, the device becomes confused and sometimes crashes completely

# to do:
#  simple scans or steps with and without triggering
#  data acquisition

def reversed_dict(dct):
    """
    Helper function.
    """
    rev = {v:k for k, v in dct.iteritems()}
    assert len(rev) == len(dct)
    return rev

class EC301(object):
    """
    Driver class for the SRS EC301 potentiostat.

    For now only implements software-triggered direct control 
    of potential or current, with no data acquisition.

    Properties (r):
    voltage: single voltage reading
    current: single current reading

    Properties (r/w):
    mode:       Control loop mode, 'POTENTIOSTAT, 'GALVANOSTAT' or 'ZRA'
    enabled:    Whether or not the cell is under control of the device
    range:      Current range setting expressed as log(range/A), so -3 is 1mA, 
                -4 is 100uA, etc, in the range -9 ... 0
    autorange:  Whether or not autoranging is on, only relevant to 
                potentiostat mode.
    bandwidth:  Control loop bandwidth in log(BW/Hz) so 3 is 1kHz, 1 is 10Hz, 
                etc, in the range 1 ... 6.
    averaging:  The number of data points averaged for each measurement, in
                the range 1, 2, 4, ..., 256 (where 256 gives 1 ms averaging).
    """

    # map from the device's mode code to clear text
    MODE_MAP = {0: 'POTENTIOSTAT', 1: 'GALVANOSTAT', 2: 'ZRA'}

    # map from current range code to log(range/A), p. 89 of the manual
    RANGE_MAP = {i:-(i-1) for i in range(1, 10+1)}

    # map from control loop bandwidth code to log(BW/hz), p. 79 of the manual
    BANDWIDTH_MAP = {i: 6-i for i in range(5+1)}

    def __init__(self, host='b-nanomax-ec301-0', port=1680, 
                 timeout=1.0, keep_socket_open=True, socket_delay=.02):
        """
        Constructor parameters:
        host (str):                 hostname or IP
        port (int):                 port number
        timeout (float):            timeout on reading from socket, mostly affects bad commands
        keep_socket_open (bool):    whether to use a persistent socket or to open disposable ones for each command
        socket_delay (float):       delay after socket commands, this avoids jamming the device
        """
        assert type(host) == str
        assert type(port) == int
        self.host = host
        self.port = port
        self.timeout = float(timeout)
        self.keep_socket_open = keep_socket_open
        self.socket_delay = socket_delay

        if self.keep_socket_open:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))

    def __del__(self):
        print 'destroying EC301 object'
        if self.keep_socket_open:
            print 'closing persistent socket'
            self.socket.shutdown(socket.SHUT_RDWR)
            self.socket.close()

    def _query(self, cmd):
        """
        Sends commands to EC301, listening for and returning 
        response from the device only if questions are asked.
        Note that you can send multiple commands separated by
        semicolons.
        """
        # open disposable socket or use persistent one
        if self.keep_socket_open:
            s = self.socket
        else:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((self.host, self.port))

        # send the command
        s.send(cmd + '\n')

        # listen to response only if we asked a question
        answer = None
        if '?' in cmd:
            # the EC301 doesn't return anything if the command is bad,
            # so we have to avoid infinite blocking here using select.
            ready = select.select([s], [], [], self.timeout)
            if ready[0]:
                answer = s.recv(100).strip().strip()

        # close disposable sockets
        if not self.keep_socket_open:
            s.shutdown(socket.SHUT_RDWR)
            s.close()
        
        # safety sleep        
        #time.sleep(self.socket_delay)

        return answer

    ##################
    ### Properties ###
    ##################

    ### Single voltage measurement
    @property
    def voltage(self):
        return float(self._query('vlevel?'))

    ### Single current measurement
    @property
    def current(self):
        return float(self._query('ilevel?'))

    ### Control mode
    @property
    def mode(self):
        return self.MODE_MAP[int(self._query('ecmode?'))]

    @mode.setter
    def mode(self, mode):
        assert mode in self.MODE_MAP.values()
        self._query('ecmode %d' % reversed_dict(self.MODE_MAP)[mode])

    ### Cell enabled? Querying gives false if the front panel switch is out.
    @property
    def enabled(self):
        return bool(int(self._query('cellon?')))

    @enabled.setter
    def enabled(self, state):
        assert val in [0, 1, True, False]
        self._query('ceenab %d' % int(state))

    ### Current range as log(range/A)
    @property
    def range(self):
        return self.RANGE_MAP[int(self._query('irange?'))]

    @range.setter
    def range(self, rng):
        assert rng in self.RANGE_MAP.values()
        target = reversed_dict(self.RANGE_MAP)[rng]
        self._query('irange %d' % target)

    ### Autorange on/off
    @property
    def autorange(self):
        return bool(int(self._query('irnaut?')))

    @autorange.setter
    def autorange(self, val):
        assert val in [0, 1, True, False]
        self.mode = 'POTENTIOSTAT'
        self._query('irnaut %d' % int(val))

    ### Sample averaging
    @property
    def averaging(self):
        return 2**int(self._query('avgexp?'))

    @averaging.setter
    def averaging(self, avg):
        assert avg in 2**np.arange(8+1, dtype=int)
        val = int(np.log2(avg))
        self._query('avgexp %d' % val)
        time.sleep(.030) # The manual specifies this.

    ### Control loop bandwidth
    @property
    def bandwidth(self):
        return self.BANDWIDTH_MAP[int(self._query('clbwth?'))]

    @bandwidth.setter
    def bandwidth(self, bw):
        assert bw in self.BANDWIDTH_MAP.values()
        target = reversed_dict(self.BANDWIDTH_MAP)[bw]
        self._query('clbwth %d' % target)
        

    ################
    ### Commands ###
    ################

    def setPotential(self, pot):
        """
        Apply a potential without setting up a scan, no data acquisition.
        Doesn't set the range, so remember to set it or to enable autorange.
        """
        try:
            if not self.mode == 'POTENTIOSTAT':
                self._query('ecmode 0; ceenab 1')
            val = float(self._query('ceenab 1; setvol %d; setvol?' % int(round(pot * 1000)))) / 1000
        except:
            val = None
            raise
        return val

    def setCurrent(self, cur):
        """
        Apply a current without setting up a scan, no data acquisition.
        Sets the range based on the requested current.
        """
        try:
            self.autorange = False
            if not self.mode == 'GALVANOSTAT':
                self._query('ecmode 1; ceenab 1')
            irange = np.ceil(np.log10(abs(cur/1.5)))
            self.range = irange
            fraction = cur / 10**irange
            result = self._query('ceenab 1; setcur %f; setcur?' % fraction)
            val = float(result) * 10**irange
        except:
            val = None
        return val

ec301 = EC301(keep_socket_open=True)
