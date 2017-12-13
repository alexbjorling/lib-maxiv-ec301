"""
Example usage and test cases for the EC301 driver library.
"""

from EC301 import EC301
import time

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
    if trg:
        plt.figure(); plt.plot(t, aux)
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
    if trg:
        plt.figure(); plt.plot(t, aux)
    plt.show()

def example_usage_acquire(trg=False):
    """
    An illustration and test of acquiring data at constant potential.
    """
    ec301 = EC301(debug=True)
    ec301.autorange = True
    ec301.setPotential(.2)
    time.sleep(.1)
    ec301.acquire(time=10.0, trigger=trg)

    while not ec301.stream.done:
        time.sleep(.1)
    
    t, E, I, aux, raw = ec301.readout()
    import matplotlib.pyplot as plt
    plt.figure(); plt.plot(t, E)
    plt.figure(); plt.plot(t, I)
    plt.show()

if __name__ == '__main__':
    example_usage_step(trg=False)

