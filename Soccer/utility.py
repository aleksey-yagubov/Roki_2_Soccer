"""
Module provides random helpers shared by the runtime code.
"""


import math
from random import getrandbits



def randrange( start, stop=None):
    """
    helper function for working with random bit sequence
    """
    if stop is None:
        stop = start
        start = 0
    upper = stop - start
    bits = 0
    pwr2 = 1
    while upper > pwr2:
        pwr2 <<= 1
        bits += 1
    while True:
        r = getrandbits(bits)
        if r < upper:
            break
    return r + start

def random():
    """
    getting a random number from 0 to 1
    """
    return randrange(10000) / 10000

def gauss( mu, sigma):
    """
    #getting a random number from Gaussian distribution
    """
    x2pi = random() * math.pi * 2
    g2rad = math.sqrt(-2.0 * math.log(1.0 - random()))
    z = math.cos(x2pi) * g2rad
    return mu + z * sigma

def gaussian( x, sigma):
    """
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    """
    return math.exp(-(x ** 2) / 2*(sigma ** 2)) / math.sqrt(2.0 * math.pi * (sigma ** 2))

