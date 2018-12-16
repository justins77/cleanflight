import os
import subprocess
import time

while True:
    print 'waiting for a device at /dev/ttyACM0...'
    while not os.path.exists('/dev/ttyACM0'):
        time.sleep(0.5)
    print 'found device'
    time.sleep(1.5)
    print 'connecting'

    subprocess.call([
        'python',
        'miniterm_csync.py',
        '/dev/ttyACM0',
        '115200'])
