#!/usr/bin/env python


import os, sys

open(sys.argv[1]).readlines()
x=[]
y=[]

for line in open(sys.argv[1]).readlines():

	line=line.split()
	x.append(float(line[0]))
	y.append(float(line[1]))
print "start_index: 0"
print "waypoints: "
str="   x: [ "
for i in range(len(x)):
	str= str +' %2.2f,'%(x[i])

str = str[:-1] + "]"
print(str)

str ="   y: [ "
for i  in range(len(y)):
	str = str + ' %2.2f,'%(y[i])


str = str[0:-1] + "] "
print(str)
str = "   type: [ "
for i in range(len(y)):
	if i==14 or i==34 or i==50 or i==66 or i==77 or i==89 or i==99 or i==118:
		str = str + "1, "
	else:
		str = str + "0, "

str = str[0:-1] + "] "
print(str)

	





