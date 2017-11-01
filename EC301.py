import socket
import select
import time
import numpy as np
import struct
from Stream import Stream

# to do:
#   * investigate why it doesn't trigger
#   * get rid of CV workaround (below, emailed SRS)
#   * why is there so much current noise?
#   * the current setup is fine for bursts, but for fly scanning we need continuous triggering, so either re-arm between shots (bad idea, will require longer latencies) or do acquisitions over the full line and sample the trigger on aux.

# sardana outline:
#   * like with the PI/scancontrol device, use different controllers simple potential/current application and for scans
#       * ec301_ctrl: motors connected to setPotential and setCurrent (with controller attributes for range etc), pstat/gstat mode can be switched automatically if needed
#       * ec301_acq_ctrl: 1d exp channels echem_E, echem_I, echem_aux, echem_raw, echem_running (or filter that out?). One of these will be the master axis that starts the scan.
#   * the type of scan and the parameters has to be configured before acquisition, with controller attributes

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
    id:      device identification string
    error:   the last error message issued
    running: is the device running a scan program?
    armed:   is the device armed and waiting for triggers?

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

    Commands:
    setPotential:   Set and hold a constant potential with no scan program
    setCurrent:     Set and hold a constant current with no scan program
    potentialStep:  Run a potential step experiment and record the data
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

        # Always keep a Stream instance to allow checking the done state
        self.stream = Stream(self)

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

    ############################
    ### Read-only properties ###
    ############################

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

    ### Device ID string
    @property
    def id(self):
        return self._query('*IDN?')

    ### Last error message
    @property
    def error(self):
        err = self._query('errlst?')
        return self._query('errdcd? ' + err)

    ### Running
    @property
    def running(self):
        raise NotImplementedError # this command doesn't behave, device always gives true. Emailed SRS.
        return bool(int(self._query('scntrg?')))

    ### Armed
    @property
    def armed(self):
        return bool(int(self._query('trgarm?')))

    #############################
    ### Read/write properties ###
    #############################    

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

    def potentialStep(self, t0=1, t1=1, E0=0, E1=1, trigger=False, 
                full_bandwidth=True, return_to_E0=True):
        """
        Carry out a potential step experiment, first hold at E0 for t0 
        seconds, then step to E1 and hold for t1 seconds.
        """

        # checks and settings
        if not self.mode == 'POTENTIOSTAT':
            self.mode = 'POTENTIOSTAT'
        if full_bandwidth:
            self.bandwidth = 6

        # format the parameters
        t0 = int(round(t0 / 4e-6))
        t1 = int(round(t1 / 4e-6))
        E0 = int(round(E0 * 1000))
        E1 = int(round(E1 * 1000))
        Efinal = {True: E0, False: E1}[return_to_E0]
        trgcode = {True: 1, False: 0}[trigger]

        # the step program interface is very hard to understand. after
        # much playing around it seems you have to add more points than
        # you think. this works.
        self._query('plinit')
        self._query('scantp 1')
        self._query('scanem 1')
        self._query('psteps 0')
        self._query('plimit 1')
        self._query('ppoint 3')
        self._query('pdatap 0 %d' % E0)
        self._query('pholdt 0 %d' % t0)
        self._query('pdatap 1 %d' % E1)
        self._query('pholdt 1 %d' %t1) #
        self._query('pincrm 1 1 1')
        self._query('pdatap 2 %d' % Efinal)
        self._query('pholdt 2 10')
        self._query('pincrm 2 1 1')
        self._query('pdatap 3 %d' % Efinal)
        self._query('pholdt 3 10')
        self._query('pincrm 3 0 0')
        self._query('plendm 1')
        self._query('pprogm?')

        # make a stream instance and start recording
        self.stream = Stream(self, complete_scan=True, max_data_points=None, debug=self.do_debug)
        self.stream.start()

        # start the scan
        self._query('pstart %d' % trgcode)

    def potentialCycle(self, t0=1, E0=.2, E1=1, E2=0, v=.100, cycles=1, trigger=False):
        """
        Carry out a CV experiment. 

        There is an apparent bug in the API here, as the device gets
        stuck in the scanning state. There is a workaround, calling
        stop() once the data collection is done. See example usage.

        There is always a short break in potential control, apparently 
        that's built into the 'ramprs' command of the API and there's 
        no obvious way around it.
        
        t0, E0: Initial hold, time and potential
        E1:         First vertex, determines the initial scan direction
        E2:         Second vertex
        v:          Scan rate
        cycles:     How many times to arrive at E2
        trigger:    True for hardware triggered scans 
        """

        # checks and settings
        if not self.mode == 'POTENTIOSTAT':
            self.debug('Device not in potentiostat mode, setting this.')
            self.mode = 'POTENTIOSTAT'

        # format the parameters
        assert cycles > 0
        rate = int(round(v / 100e-6))
        hold = int(round(t0 / 100e-6))
        E0 = int(round(E0 * 1000))
        E1 = int(round(E1 * 1000))
        E2 = int(round(E2 * 1000))
        trgcode = {True: 4, False: 0}[trigger]

        # write the program
        self._query('ramprs')
        if cycles > 1:
            self._query('scantp 0')
            self._query('rampcy %d' % (cycles-1))
        else:
            self._query('scantp 1')
        self._query('scanem 1')
        self._query('ramppt 0 %d' % E0)
        self._query('ramppt 1 %d' % E1)
        self._query('ramppt 2 %d' % E2)
        self._query('ramprt 0 %d' % rate)
        self._query('ramprt 1 %d' % rate)
        self._query('rampdt 0 %d' % hold)
        self._query('rampdt 1 0')
        self._query('rampdt 2 0')
        self._query('ramppg? 0')

        # make a stream instance and start recording
        self.stream = Stream(self, complete_scan=True, max_data_points=None, debug=self.do_debug)
        self.stream.start()

        # start the scan
        self._query('rampst %d' % trgcode)

    def stop(self):
        """
        Stop any of the implemented scan programs.
        """
        self._query('plstop')
        self._query('rampen')
    
    #######################################################
    ### Illustrations and tests, not for Tango exposure ###
    #######################################################

    def example_step(self):
        """
        An illustration and test of the potential step program and data acquisition.
        """
        self.setPotential(-.1)
        self.averaging=256
        time.sleep(2)
        self.range = -5
        self.potentialStep(t0=2, t1=3, E0=.02, E1=.05, return_to_E0=True, trigger=False)
        t0 = time.time()
        while not self.stream.done:
            print 'data points so far: %d' % len(self.stream.E)
            # illustrating how an acquisition can be stopped:
            if time.time() - t0 > 30:
                print 'stopping'
                # you can just cancel the stream (the program then continues)
                #self.stream.cancel = True
                # or you can stop the scan (the stream then detects this and also finishes)
                self.stop()
            time.sleep(.1)
        E = self.stream.E
        I = self.stream.I
        aux = self.stream.I
        print len(E), len(I)
        print self.error

        import matplotlib.pyplot as plt
        t = np.arange(len(E)) * 4e-6 * 256
        plt.figure(); plt.plot(t, E)
        plt.figure(); plt.plot(t, I)
        plt.show()
        return E, I, aux

    def example_cv(self):
        """
        An illustration and test of the potential step program and data acquisition.
        """
        self.setPotential(-.1)
        self.averaging=256
        time.sleep(2)
        self.range = -3
        self.potentialCycle(v=.300, E0=.05, E1=.8, E2=-.2, cycles=1)
        while not self.stream.done:
            print 'data points so far: %d' % len(self.stream.E)
            time.sleep(.1)
        # there's a bug in the CV protocol (emailed SRS about this),
        # the device doesn't leave its scanning mode. This means you
        # can't set a new potential afterwards, although the scanning
        # bit in the binary stream does switch off. This is the workaround:
        self.stop()             #
        self.setPotential(.05)  #
        #########################
        E = self.stream.E
        I = self.stream.I
        print len(E), len(I)
        print self.error

        import matplotlib.pyplot as plt
        t = np.arange(len(E)) * 4e-6 * 256
        plt.figure(); plt.plot(t, E)
        plt.figure(); plt.plot(t, I)
        plt.show()
        return E, I

if __name__ == '__main__':        
    """
    Example usage.
    """
    ec301 = EC301(debug=False)
#    ec301.example_step()
    ec301.example_cv()

