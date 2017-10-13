import socket
import select
import time
import numpy as np

# to do:
#  simple scans or steps with and without triggering

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
    aux:     single reading of the three auxiliary channels,
             returned as a length-3 array.

    Properties (r/w):
    mode:       Control loop mode, 'POTENTIOSTAT, 'GALVANOSTAT' or 'ZRA'
    enabled:    Whether or not the cell is under control of the device
    range:      Current range setting expressed as log(range/A), so -3 is 1mA, 
                -4 is 100uA, etc, in the range -9 ... 0
    autorange:  Whether or not autoranging is on, only relevant to 
                potentiostat mode.
    bandwidth:  Control loop bandwidth in log(BW/Hz) so 3 is 1kHz, 1 is 10Hz, 
                etc, in the range 1 ... 6. Set this lower to obtain smooth
                applied potentials (CV), set it high for fast changes (step
                or fast scans).
    averaging:  The number of 4us data points averaged for each measurement, in
                the range 1, 2, 4, ..., 256 (where 256 gives ~1 ms averaging).
    """

    # map from the device's mode code to clear text
    MODE_MAP = {0: 'POTENTIOSTAT', 1: 'GALVANOSTAT', 2: 'ZRA'}

    # map from current range code to log(range/A), p. 89 of the manual
    RANGE_MAP = {i:-(i-1) for i in range(1, 10+1)}

    # map from control loop bandwidth code to log(BW/hz), p. 79 of the manual
    BANDWIDTH_MAP = {i: 6-i for i in range(5+1)}

    def __init__(self, host='b-nanomax-ec301-0', port=1680, 
                 timeout=1.0, debug=False):
        """
        Constructor parameters:
        host (str):                 hostname or IP
        port (int):                 port number
        timeout (float):            timeout on reading from socket, mostly affects bad commands
        """
        assert type(host) == str
        assert type(port) == int
        self.host = host
        self.port = port
        self.timeout = float(timeout)
        self.do_debug = debug

        # Keep a persistent socket. If you don't then the device
        # very easily gets jammed from repeated commands.
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))

    def __del__(self):
        self.socket.shutdown(socket.SHUT_RDWR)
        self.socket.close()

    def debug(self, msg):
        if self.do_debug:
            print msg

    def _query(self, cmd, bytes=100):
        """
        Sends commands to EC301, listening for and returning 
        response from the device only if questions are asked.
        Note that you can send multiple commands separated by
        semicolons.
        """
        s = self.socket

        # send the command
        s.send(cmd + '\n')
        self.debug("Sent cmd '%s'" % cmd)

        # listen to response only if we asked a question
        answer = None
        if '?' in cmd:
            # the EC301 doesn't return anything if the command is bad,
            # so we have to avoid infinite blocking here using select.
            ready = select.select([s], [], [], self.timeout)
            if ready[0]:
                answer = s.recv(bytes).strip().strip()
        self.debug("Got response '%s'" % answer)

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

    ### Single auxiliary input measurements
    @property
    def aux(self):
        return np.array(map(float, self._query('getaux? 4').split(', ')))

    ### Control mode
    @property
    def mode(self):
        return self.MODE_MAP[int(self._query('ecmode?'))]

    @mode.setter
    def mode(self, mode):
        assert mode in self.MODE_MAP.values()
        if not mode == 'POTENTIOSTAT':
            self.autorange = False
        self._query('ecmode %d' % reversed_dict(self.MODE_MAP)[mode])

    ### Cell enabled? Querying gives false if the front panel switch is out.
    @property
    def enabled(self):
        return bool(int(self._query('cellon?')))

    @enabled.setter
    def enabled(self, state):
        assert state in [0, 1, True, False]
        self._query('ceenab %d' % int(state))

    ### Current range as log(range/A)
    @property
    def range(self):
        return self.RANGE_MAP[int(self._query('irange?'))]

    @range.setter
    def range(self, rng):
        assert rng in self.RANGE_MAP.values()
        if self.autorange:
            self.autorange = False
        target = reversed_dict(self.RANGE_MAP)[rng]
        self._query('irange %d' % target)

    ### Autorange on/off
    @property
    def autorange(self):
        return bool(int(self._query('irnaut?')))

    @autorange.setter
    def autorange(self, val):
        assert val in [0, 1, True, False]
        if not self.mode == 'POTENTIOSTAT':
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

    ### Last error message
    @property
    def error(self):
        err = self._query('errlst?')
        return self._query('errdcd? ' + err)
        

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
            if np.isclose(cur, 0, atol=1e-9):
                irange = min(self.RANGE_MAP.values())
            else:
                irange = np.ceil(np.log10(abs(cur/1.5)))
            self.range = irange
            fraction = cur / 10**irange
            result = self._query('ceenab 1; setcur %f; setcur?' % fraction)
            val = float(result) * 10**irange
        except:
            val = None
        return val

    def receive(self):
        """
        Test method to receive a stream, should be threaded etc.
        """
        # start streaming
        self._query('getbda 1')

        try:
            while True:
                print self.socket.recv(100).strip().strip()
        except KeyboardInterrupt:
            self._query('getbda 0')

    def potentialStep(self, t0, t1, E0, E1, trigger=False, full_bandwidth=True):
        """
        Carry out a potential step experiment, first hold at E0 
        for t0 seconds, then step to E1 and hold for t1 seconds.
        """

        # checks and settings
        if not self.mode == 'POTENTIOSTAT':
            self.mode = 'POTENTIOSTAT'
        if full_bandwidth:
            self.bandwidth = 6

        t0 = int(round(t0 / 4e-6))
        t1 = int(round(t1 / 4e-6))
                                        
        # there's something wrong here, 
        self._query('plinit')
        self._query('ppoint 1')
        self._query('psteps 50')
        self._query('plimit 1')
        self._query('pdatap 0 %d' % int(round(E0 * 1000)))
        self._query('pholdt 0 %d' % t0)
        self._query('pdatap 1 %d' % int(round(E1 * 1000)))
        self._query('pholdt 1 %d' % t1)
        self._query('pincrm 1 0 0')
        self._query('plendm 0')
        self._query('pprogm?')
        self._query('pstart 0') # this is by trial and error, the API says no arguments
        
ec301 = EC301(debug=False)
#ec301.setPotential(-.1)
#time.sleep(2)
#ec301.potentialStep(2, 2, .45, .65)
#print ec301.error

