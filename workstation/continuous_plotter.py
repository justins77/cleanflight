from collections import OrderedDict
import math
import threading
import time
import re

import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt

class Series(object):
    def __init__(self, name):
        self.name = name
        self.xs = []
        self.ys = []


MAX_HISTORY = 100

class ContinuousPlotter(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.series_map = OrderedDict()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ani = animation.FuncAnimation(self.fig, self._animation_function, interval=100)
        self.has_new_data = True

    def _animation_function(self, unused):
        if not self.has_new_data:
            return
        self.has_new_data = False

        self.ax.clear()
        self.ax.grid(True)
        with self.lock:
            legend = []
            for series in self.series_map.values():
                self.ax.plot(series.xs, series.ys, marker='o')
                legend.append(series.name)
            self.ax.legend(legend, loc='upper left')

    def _get_series(self, name):
        if name in self.series_map:
            return self.series_map[name]
        series = Series(name)
        self.series_map[name] = series
        return series

    def append_now(self, name, y):
        self.append(name, time.time(), y)

    def append(self, name, x, y):
        with self.lock:
            series = self._get_series(name)
            series.xs = series.xs[-MAX_HISTORY:]
            series.ys = series.ys[-MAX_HISTORY:]
            series.xs.append(x)
            series.ys.append(y)
        self.has_new_data = True

    def run(self):
        plt.show()

    def start(self):
        thread = threading.Thread(target=self._run)
        thread.daemon = True
        thread.start()


class CsyncContinuousPlotter(ContinuousPlotter):
    def __init__(self):
        super(CsyncContinuousPlotter, self).__init__()
        self.buffer = ''

    def write(self, text):
        self.buffer += text
        while True:
            cr_pos = self.buffer.find('\n')
            if cr_pos < 0:
                break
            line = self.buffer[0:cr_pos]
            self.buffer = self.buffer[cr_pos+1:]
            self.process_line(line.strip())

    def process_line(self, line):
        match = re.match(r'clockDeltaData ([0-9\-]+),([0-9\-]+),([0-9\-]+),([0-9\-]+),([0-9\-]+),([0-9\-]+)$', line)
        if match:
            publish_time = float(match.group(1)) / 1e6
            #self.append('ours', publish_time, int(match.group(2)))
            #self.append('theirs', publish_time, int(match.group(3)))
            #self.append('tx_time', publish_time, int(match.group(4)))
            self.append('cycle_start_delta', publish_time, int(match.group(5)))
            self.append('peer_cycle_start_delta', publish_time, int(match.group(6)))


# For testing only
def main_loop(plotter):
    i = 0
    while True:
        plotter.append('sin', time.time(), math.sin(i/10.))
        plotter.append('cos', time.time(), math.cos(i/10.))
        time.sleep(0.1)
        i += 1


def main():
    plotter = ContinuousPlotter()

    thread = threading.Thread(target=main_loop, args=(plotter,))
    thread.daemon = True
    thread.start()

    plotter.run()


if __name__ == '__main__':
    main()
