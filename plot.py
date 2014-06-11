#!/usr/bin/python

from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt

from optparse import OptionParser

usage = "usage: %prog [options] arg"

parser = OptionParser()
parser = OptionParser(usage=usage)
parser.add_option("-l", "--link", dest="link", default="fore_arm_left")

(options, args) = parser.parse_args()
if len(args) != 1:
	parser.print_usage(file=None)
	exit()

fname = args[0]
data = [];
start_time = None

with open(fname) as f:
	for line in f.readlines():
		if line.startswith('rp_'):
			a = line.replace('\n','').split(' ')

			t = int(a[1]) / 1000000.0 # a[1] is in ms
			if not start_time:
				start_time = t
			t = t - start_time

			#t = int(a[1]) / 1000000.0 # a[1] is in ms

			d = {
				'type':	a[0],
				'time':	t,
				'name': a[2],

				# pointOnA
				'p0':	a[3],
				'p1':	a[4],
				'p2':	a[5],
				'p': [a[3], a[4], a[5]],

				# direction
				'n0':	a[6],
				'n1':	a[7],
				'n2':	a[8],
				'n': [a[6], a[7], a[8]],

				# amplitude
				'a': [a[9]],
			}
			data.append(d)

time_min = min([d['time'] for d in data])
time_max = max([d['time'] for d in data])

data_bt   = [d for d in data if d['type'] == 'rp_bt']
data_temp = [d for d in data if d['type'] == 'rp_temp']
data_fcl  = [d for d in data if d['type'] == 'rp_fcl']

link = options.link
print 'using for link ', link

t_bt  = [d['time'] for d in data_bt if d['name'] == link]
p_bt  = [d['p']    for d in data_bt if d['name'] == link]

t_temp  = [d['time'] for d in data_temp if d['name'] == link]
p_temp  = [d['p']    for d in data_temp if d['name'] == link]

t_fcl  = [d['time'] for d in data_fcl if d['name'] == link]
p_fcl  = [d['p']    for d in data_fcl if d['name'] == link]

a_bt   = [d['a'] for d in data_bt   if d['name'] == link]
a_temp = [d['a'] for d in data_temp if d['name'] == link]
a_fcl  = [d['a'] for d in data_fcl  if d['name'] == link]

axis = plt.subplot(4,1,1)
plt.plot(t_bt, p_bt)
plt.xlim(time_min, time_max)
plt.legend(['x', 'y', 'z'])

axis = plt.subplot(4,1,2)
plt.plot(t_temp, p_temp)
plt.xlim(time_min, time_max)
plt.legend(['x', 'y', 'z'])

axis = plt.subplot(4,1,3)
plt.plot(t_fcl, p_fcl)
plt.xlim(time_min, time_max)
plt.legend(['x', 'y', 'z'])

axis = plt.subplot(4,1,4)
plt.plot(
	t_bt,   a_bt,
	t_temp, a_temp,
	t_fcl,  a_fcl)
plt.xlim(time_min, time_max)
plt.legend(['bullet', 'temp', 'fcl'])

plt.show()
