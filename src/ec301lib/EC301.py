import socket
import select
import time
import numpy as np
import struct
from Stream import Stream

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

    Implements direct control of current and potential, potential
    step and cycle scans, and data acquisition. All hardware-triggered
    functionality relies on the trigger signal also being connected
    to the synchronous ADC input for filtering. Trigger pulses should 
    therefore not be shorter than the data acquisition period 
    (4 us - 1 ms depending on settings).

    Properties (r):
    voltage: single voltage reading
    current: single current reading
    aux:     single reading of the three auxiliary channels,
             returned as a length-3 array.
    id:      device identification string
    error:   the last error message issued
    running: is the device running a scan program?

    Properties (r/w):
    mode:       Control loop mode, 'POTENTIOSTAT, 'GALVANOSTAT' or 'ZRA'
    enabled:    Whether or not the cell is under control of the device
    Irange:     Current range setting expressed as log(range/A), so -3 is 1mA, 
                -4 is 100uA, etc, in the range -9 ... 0
    Erange:     Potential ADC range setting in Volts, 2, 5, or 15.
    autorange:  Whether or not autoranging is on, only relevant to 
                potentiostat mode.
    bandwidth:  Control loop bandwidth in log(BW/Hz) so 3 is 1kHz, 1 is 10Hz, 
                etc, in the range 1 ... 6. Set this lower to obtain smooth
                applied potentials (CV), set it high for fast changes (step
                or fast scans).
    averaging:  The number of 4us data points averaged for each measurement, in
                the range 1, 2, 4, ..., 256 (where 256 gives ~1 ms averaging).
    Ilowpass:   Whether or not to apply the 10 kHz anti-aliasing filter in 
                front of the current ADC.
    Elowpass:   Whether or not to apply the 10 kHz anti-aliasing filter in 
                front of the voltage ADC.
    compliance_limit: Limit on the compliance voltage in V

    Commands:
    setPotential:   Set and hold a constant potential with no scan program
    setCurrent:     Set and hold a constant current with no scan program
    acquire:        Acquire data without applying a scan program
    potentialStep:  Run a potential step experiment and record the data
    potentialCycle: Run a Cyclic voltammetry experiment and record the data
    stop:           Stop any ongoing scans
    """

    # map from the device's mode code to clear text
    MODE_MAP = {0: 'POTENTIOSTAT', 1: 'GALVANOSTAT', 2: 'ZRA'}

    # map from current range code to log(range/A), p. 89 of the manual
    IRANGE_MAP = {i:-(i-1) for i in range(1, 10+1)}

    # map from voltage range code to actual range in volts
    ERANGE_MAP = {0: 2, 1: 5, 2: 15}

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

        # General initialization
        self.enabled = False
        self._query('BSTREN 0')     # disable booster
        self._query('LPFILE 0')     # disable front panel E low-pass filter
        self._query('LPFILI 0')     # disable front panel I low-pass filter
        self.Erange = 2             # +- 2V
        self._query('AUTOTB 0')     # use only internal timebase
        self.Elowpass = True        # 10 kHz filter on E
        self.Ilowpass = True        # 10 kHz filter on I
        self._query('IRENAB 0')     # disable IR compensation
        self._query('IRTYPE 1')     # positive feedback
        self._query('CELIMT 1')     # enable compliance limit
        self.compliance_limit = 3.0 # compliance voltage limit +- 3V
        self._query('BRENAB 0')     # bias rejection off
        self._query('DCNTRL 0')     # direct control off
        self._query('ADDSCN 0')     # ignore external input voltage
        self.enabled = True

        # Always keep a Stream instance to allow checking the done state
        self.stream = Stream(self)
        self.stream.done = True # means not running

        # Stop any lingering scans
        self.stop()

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

        # if a stream is running, it makes no sense to ask a question
        if '?' in cmd and self.running:
            return None

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

    def _run_txt(self, filename):
        """
        Test method for running programs from text files.
        """
        with file(filename) as fp:
            for line in fp:
                if not line.strip().startswith('#'):
                    self._query(line.strip().split('#')[0])

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
        return not self.stream.done

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
    def Irange(self):
        return self.IRANGE_MAP[int(self._query('irange?'))]

    @Irange.setter
    def Irange(self, rng):
        assert rng in self.IRANGE_MAP.values()
        if self.autorange:
            self.autorange = False
        target = reversed_dict(self.IRANGE_MAP)[rng]
        self._query('irange %d' % target)

    ### Potential range in Volts
    @property
    def Erange(self):
        return self.ERANGE_MAP[int(self._query('eadcrg?'))]

    @Erange.setter
    def Erange(self, rng):
        assert rng in self.ERANGE_MAP.values()
        target = reversed_dict(self.ERANGE_MAP)[rng]
        self._query('eadcrg %d' % target)

    ### Current autorange on/off
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

    ### I ADC low-pass filter
    @property
    def Ilowpass(self):
        return int(self._query('iadcfl?')) == 2

    @Ilowpass.setter
    def Ilowpass(self, val):
        assert val in [0, 1, True, False]
        code = {True: 2, False: 1}[val]
        self._query('iadcfl %d' % code)

    ### E ADC low-pass filter
    @property
    def Elowpass(self):
        return int(self._query('eadcfl?')) == 2

    @Elowpass.setter
    def Elowpass(self, val):
        assert val in [0, 1, True, False]
        code = {True: 2, False: 0}[val]
        self._query('eadcfl %d' % code)

    ### Compliance voltage limit
    @property
    def compliance_limit(self):
        return float(self._query('celimv?')) / 1000.0

    @compliance_limit.setter
    def compliance_limit(self, val):
        value = int(round(val * 1000))
        value = max(value, 500)
        value = min(value, 30000)
        self._query('celimv %d' % value)

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
        Sets the Irange based on the requested current.
        """
        try:
            self.autorange = False
            if not self.mode == 'GALVANOSTAT':
                self._query('ecmode 1; ceenab 1')
            if np.isclose(cur, 0, atol=1e-9):
                irange = min(self.IRANGE_MAP.values())
            else:
                irange = np.ceil(np.log10(abs(cur/1.5)))
            self.Irange = irange
            fraction = cur / 10**irange
            result = self._query('ceenab 1; setcur %f; setcur?' % fraction)
            val = float(result) * 10**irange
        except:
            val = None
        return val

    def acquire(self, time=1.0, trigger=False):
        """
        Simple data acquisition without a scan program. The time
        is rounded up to a whole number of measurement intervals.
        Using the trigger doesn't actually arm the device, but
        simply the data stream to ignore everything before the
        first trigger event.
        """
        datapoints = int(np.ceil(time / (4e-6 * self.averaging)))
        self.debug('Collecting %d data points, real time %f s'
            % (datapoints, datapoints * 4e-6 * self.averaging))
        self.stream = Stream(self, max_data_points=datapoints,
                             filter_pre_trig=trigger,
                             debug=self.do_debug)
        self.stream.start()

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
        self._query('psteps 50')
        self._query('plimit 1')
        self._query('ppoint 3')
        self._query('pdatap 0 %d' % E0)
        self._query('pholdt 0 %d' % t0)
        self._query('pdatap 1 %d' % E1)
        self._query('pholdt 1 %d' %t1) #
        self._query('pincrm 1 0 0')
        self._query('pdatap 2 %d' % Efinal)
        self._query('pholdt 2 10')
        self._query('pincrm 2 0 1')
        self._query('pdatap 3 %d' % Efinal)
        self._query('pholdt 3 10')
        self._query('pincrm 3 0 1')
        self._query('plendm 1')
        self._query('pprogm?')

        # make a stream instance and start recording
        self.stream = Stream(self, complete_scan=True, max_data_points=None,
                             filter_pre_scan=True, filter_pre_trig=trigger,
                             debug=self.do_debug)
        self.stream.start()
        time.sleep(.1) # avoids missing data

        # start the scan
        self._query('trgarm 0')
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
        self._query('ramppt 0 %d' % E0)
        self._query('ramppt 1 %d' % E1)
        self._query('ramppt 2 %d' % E2)
        self._query('rampdt 0 %d' % hold)
        self._query('rampdt 1 0')
        self._query('rampdt 2 0')
        self._query('ramprt 0 %d' % rate)
        self._query('ramprt 1 %d' % rate)
        self._query('scanem 1')
        self._query('ramppg? 0')

        # make a stream instance and start recording
        self.stream = Stream(self, complete_scan=True, max_data_points=None,
                             filter_pre_scan=True, filter_pre_trig=trigger,
                             debug=self.do_debug)
        self.stream.start()
        time.sleep(.1) # avoids missing data

        # start the scan
        self._query('trgarm 0')
        self._query('rampst %d' % trgcode)

    def stop(self):
        """
        Stop any of the implemented scan programs.
        """
        self._query('plstop')
        self._query('rampen')
        # acquire() doesn't listen to scan state, so needs the stream cancelled
        self.stream.cancel = True

    def readout(self):
        """
        Get the last data captured. 

        Returns:
        t:      time
        E:      potential
        I:      current
        aux:    synchronous ADC input, used to monitor the trigger
        raw:    4-byte bit representation of the state at each data point        
        """
        t = np.arange(len(self.stream.E)) * 4e-6 * self.averaging
        return t, self.stream.E, self.stream.I, self.stream.aux, self.stream.raw
    

def example_usage_step(trg=False):
    """
    An illustration and test of the potential step program and data acquisition.
    """
    ec301 = EC301(debug=True)

    ec301.setPotential(-.1)
    ec301.averaging=256
    time.sleep(2)
    ec301.Irange = -5
    ec301.potentialStep(t0=2, t1=3, E0=.02, E1=.05, return_to_E0=True, trigger=trg)

    t0 = time.time()
    while not ec301.stream.done:
        print 'data points so far: %d' % len(ec301.stream.E)
        # illustrating how an acquisition can be stopped:
        if time.time() - t0 > 10:
            print 'stopping'
            # you can just cancel the stream (the program then continues)
            #ec301.stream.cancel = True
            # or you can stop the scan (the stream then detects this and also finishes)
            ec301.stop()
        time.sleep(.1)

    t, E, I, aux, raw = ec301.readout()
    print len(E), len(I)
    print ec301.error

    import matplotlib.pyplot as plt
    plt.figure(); plt.plot(t, E)
    plt.figure(); plt.plot(t, I)
    plt.show()

def example_usage_cv(trg=False):
    """
    An illustration and test of the potential step program and data acquisition.
    """
    ec301 = EC301(debug=True)

    ec301.setPotential(-.1)
    ec301.averaging=256
    time.sleep(2)
    ec301.Irange = -3

    ec301.potentialCycle(v=.500, E0=.05, E1=.8, E2=-.2, cycles=1, trigger=trg)
    while not ec301.stream.done:
        print 'data points so far: %d, running: %s' % (len(ec301.stream.E), str(ec301.running))
        time.sleep(.1)
    # the scan doesn't actually stop at E0, but at E2. You can go back manually:
    ec301.setPotential(.05)

    t, E, I, aux, raw = ec301.readout()
    import matplotlib.pyplot as plt
    plt.figure(); plt.plot(t, E)
    plt.figure(); plt.plot(t, I)
    plt.show()

def example_usage_bias():
    """
    An illustration and test of acquiring data at constant potential.
    """
    ec301 = EC301(debug=True)
    ec301.autorange = True
    ec301.setPotential(.2)
    time.sleep(.1)
    ec301.acquire(time=2.0, trigger=False)

    while not ec301.stream.done:
        time.sleep(.1)
    
    t, E, I, aux, raw = ec301.readout()
    import matplotlib.pyplot as plt
    plt.figure(); plt.plot(t, E)
    plt.figure(); plt.plot(t, I)
    plt.show()

