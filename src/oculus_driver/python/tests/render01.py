#! /usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import argparse

from oculus_python.files import OculusFileReader
from oculus_python.utils import PingRenderer

parser = argparse.ArgumentParser(
    prog='OculusFileReader',
    description='Example of how to read and display the content of a .oculus ' +
                'file. This will display the first ping from a the file.')
parser.add_argument('filename', type=str,
                    help='Path to a .oculus file to display')
args = parser.parse_args()

print('Opening', args.filename)

f = OculusFileReader(args.filename)

msg = f.read_next_ping() # this can be called several time to iterate through the pings
                         # will return None when finished
if msg is None:
    print('File seems to be empty. Aborting.')

renderer = PingRenderer()
img = renderer.render(msg)

_, ax = plt.subplots(1,1)
ax.imshow(img)

plt.show(block=False)



