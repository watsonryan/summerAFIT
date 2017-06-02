#!/usr/bin/env python 

import os, glob, subprocess, progressbar

progress = progressbar.ProgressBar()
print('\n\n')
for filename in progress(xrange(0,0.01,10)):

    cmd = ['./../../../build/examples/processNovatel', '-g', newFile,
          '--dir', currDir, '--writeENU', '--writeECEF']

    p = subprocess.Popen(cmd, stdout=open(os.devnull, 'wb')).wait()dd
