import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import math
import threading
import time


class ContinuousPlotter(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.xs = []
        self.ys = []
        fig = plt.figure()
        self.ax = fig.add_subplot(1, 1, 1)
        self.ani = animation.FuncAnimation(fig, self._animation_function, interval=100)

    def _animation_function(self, unused):
        self.ax.clear()
        self.ax.grid(True)
        with self.lock:
            self.ax.plot(self.xs, self.ys)

    def append_now(self, y):
        self.append(time.time(), y)

    def append(self, x, y):
        with self.lock:
            self.xs = self.xs[-100:]
            self.ys = self.ys[-100:]
            self.xs.append(x)
            self.ys.append(y)

    def run(self):
        plt.show()

    def start(self):
        thread = threading.Thread(target=self._run)
        thread.daemon = True
        thread.start()


def main_loop(plotter):
    i = 0
    while True:
        plotter.append(time.time(), math.sin(i/10.))
        time.sleep(0.1)
        i += 1


def main():
    plotter = ContinuousPlotter()

    thread = threading.Thread(target=main_loop, args=(plotter,))
    thread.daemon = True
    thread.start()

    plotter.run()



main()
