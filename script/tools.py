# Copyright 2023 CNRS

# Author: Florent Lamiraux

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Function of a real value defined over intervals
#
# member segments is a list of functions defined over intervals
# each such function should have members t_start and t_end defining the
# interval of definition of the function.
class Piecewise(object):
    def __init__(self):
        self.segments = list()

    # Evaluation of the function with operator()
    def __call__(self, t):
        if len(self.segments) == 0:
            raise RuntimeError('This "Piecewise" instance is empty.')
        for s in self.segments:
            if t <= s.t_end: break
        return s(t)

# Affine trajectory
class Affine(object):
    # Constructor
    #  - [t_init, t_end]: interval of definition
    #  - p_init, p_end: initial and final values
    def __init__(self, t_init, t_end, p_init, p_end):
        if t_init >= t_end:
            raise RuntimeError(f"t_end ({t_end}) should be bigger than t_init ({t_init}).")
        self.t_init = t_init
        self.t_end = t_end
        self.p_init = p_init
        self.p_end = p_end

    # Evaluation of the function with operator()
    def __call__(self, t):
        a = (t - self.t_init)/(self.t_end-self.t_init)
        p = (1 - a) * self.p_init + a * self.p_end
        return p

# Constant function
class Constant(object):
    def __init__(self, t_init, t_end, value):
        self.t_init = t_init
        self.t_end = t_end
        self.value = value

    def __call__(self, t):
        return self.value


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np

    af1 = Affine(0, 1, 1, 2)
    af2 = Affine(1, 2, 2, 0)
    pw = Piecewise()
    pw.segments.append(af1)
    pw.segments.append(af2)
    
    times = 0.01 * np.arange(201)
    values = list(map(pw, times))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(times, values, label="piecewise affine function")
    ax.legend()
    plt.show()

        
