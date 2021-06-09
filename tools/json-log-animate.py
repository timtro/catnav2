#!/usr/bin/env python3
"""
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2013 - Timothy A.V. Teatro
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with vme-nmpc. If not, see <http://www.gnu.org/licenses/>.

Description
===========

This code shows an animation of the path generation, illustrating not only the
motion through the field, but a comparison of the SD convergence results to the
reference line which the robot should with to track.

This code is also intended as a general demonstration of the usage of the
nmpc_stats_and_quantities and nmpc_output_parse modules.

"""

import argparse

# from sys import stdin, exit
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from modules.JsonLogParser import JsonLogParser
import json


class OPoint:

    def __init__(self, x, y, epsilon, pwr):
        self.x, self.y, self.epsilon, self.pwr = x, y, epsilon, pwr

    def phi(self, x, y):
        return 1/((self.x - x)**self.pwr + (self.y - y)**self.pwr + self.epsilon)

class ONull:

    def phi(self, x, y):
        return 0

def json_to_obstacle(o):
    if o['type'] == "Point":
        return OPoint(o['x'], o['y'], o['epsilon'], o['pwr'])
    elif o['type'] == "Null":
        return ONull()
    else:
        raise Exception("Unknown object type: " + o['type'])

class AnimatedPlot:

    def __init__(self, xr, yr, pathRadius):
        self.fig = plt.figure(figsize=(11, 8.5), dpi=94, facecolor='#efefef')
        gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1])
        self.ax1 = self.fig.add_subplot(gs[0], adjustable='box', aspect=1.0)
        self.ax1.set_xlim(xr[0], xr[1])
        self.ax1.set_ylim(yr[0], yr[1])
        self.ax1.grid()
        self.ax1Path, = self.ax1.plot([], [], 'ro-', lw=3, ms=3)
        self.ax1ErrPath, = self.ax1.plot([], [], 'yo-', lw=4, ms=3)
        self.ax1ExecPath, = self.ax1.plot([], [], 'r-', lw=2)
        self.ax1meshX, self.ax1meshY = np.meshgrid(np.arange(xr[0], xr[1], 0.1),
                np.arange(yr[0], yr[1], 0.1))
        Z = np.zeros_like(self.ax1meshX)
        self.ax1Field = self.ax1.contourf(self.ax1meshX, self.ax1meshY, Z,
                levels=64)

        self.ax2 = self.fig.add_subplot(
            gs[1], adjustable='box', aspect=1.0, facecolor='w')
        plt.setp(self.ax2.get_xticklabels(), visible=False)
        plt.setp(self.ax2.get_yticklabels(), visible=False)
        self.ax2.set_xlim(-pathRadius, pathRadius)
        self.ax2.set_ylim(-pathRadius, pathRadius)
        self.ax2Path, = self.ax2.plot([], [], 'ro-', lw=2, ms=5)
        self.ax2ErrPath, = self.ax2.plot([], [], 'yo-', lw=2, ms=5)

        plt.tight_layout(pad=1.08, h_pad=None, w_pad=None, rect=None)

        self.execPath = [[], []]

    def animationInit(self):
        self.ax1Path.set_data([], [])
        self.ax1ErrPath.set_data([], [])
        self.ax2Path.set_data([], [])
        self.ax2ErrPath.set_data([], [])
        self.ax1ExecPath.set_data(self.execPath[0], self.execPath[1])
        return self.ax1Path, self.ax1ErrPath, self.ax1ExecPath, self.ax2Path, \
            self.ax2ErrPath

    def startAnimation(self, interval, updateFunction):
        self.animater = animation.FuncAnimation(
            self.fig,
            updateFunction,
            init_func=self.animationInit,
            interval=interval,
            blit=True)
        plt.show()

def updatePlotData(data):
    while True:
        jsonData = logParser.getNextObjectAsDict()
        if not jsonData:
            return aniPlot.ax1Path, aniPlot.ax1ErrPath, aniPlot.ax1ExecPath,\
                aniPlot.ax2Path, aniPlot.ax2ErrPath
        if 'ex' in jsonData.keys():
            break

    aniPlot.execPath[0].append(jsonData['x'][0])
    aniPlot.execPath[1].append(jsonData['y'][0])

    x = np.array(jsonData['x'])
    y = np.array(jsonData['y'])
    ex = np.array(jsonData['ex'])
    ey = np.array(jsonData['ey'])
    obstacles = list(map(json_to_obstacle, jsonData['obstacles']))

    Phi = np.zeros_like(aniPlot.ax1meshX)
    for each in obstacles:
        Phi += each.phi(aniPlot.ax1meshX, aniPlot.ax1meshY)

    xRelative = x - x[0]
    yRelative = y - y[0]

    xTracked = x[1:] - ex
    yTracked = y[1:] - ey

    xTrackedRelative = xTracked - x[0]
    yTrackedRelative = yTracked - y[0]

    aniPlot.ax1.collections = []
    aniPlot.ax1.contourf(aniPlot.ax1meshX, aniPlot.ax1meshY, Phi, levels=64)

    aniPlot.ax1Path.set_data(x, y)
    aniPlot.ax2Path.set_data(xRelative, yRelative)
    aniPlot.ax1ErrPath.set_data(xTracked, yTracked)
    aniPlot.ax2ErrPath.set_data(xTrackedRelative, yTrackedRelative)
    aniPlot.ax1ExecPath.set_data(aniPlot.execPath[0], aniPlot.execPath[1])


    return aniPlot.ax1Path, aniPlot.ax1ErrPath, aniPlot.ax1ExecPath,\
        aniPlot.ax2Path, aniPlot.ax2ErrPath


parser = argparse.ArgumentParser(
    description='Animate a plot of the NMPC calculation')
parser.add_argument('FILENAME',
    dest='inputFileName',
    default='',
    help='JSON formatted input file',
    metavar='FILENAME')
parser.add_argument(
    '-i',
    '--interval',
    dest='interval',
    type=float,
    default=10,
    help='Graph refresh interval',
    metavar='INTERVAL')

args = parser.parse_args()

with open(args.inputFileName, 'r') as instream:
    logParser = JsonLogParser(instream)
    aniPlot = AnimatedPlot(xr=[0, 10], yr=[-5, 5], pathRadius=2)
    aniPlot.startAnimation(
        interval=args.interval, updateFunction=updatePlotData)
