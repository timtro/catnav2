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


def xy_minmax(log):
    xmin, xmax = 0, 0
    ymin, ymax = 0, 0

    for step in log:
        thisxmin = min(step['x'])
        thisxmax = max(step['x'])
        thisymin = min(step['y'])
        thisymax = max(step['y'])

        if thisxmax > xmax:
            xmax = thisxmax
        if thisxmin < xmin:
            xmin = thisxmin
        if thisymax > ymax:
            ymax = thisymax
        if thisymin < ymin:
            ymin = thisymin

    return (xmin, xmax, ymin, ymax)


def equalize_axes(xmin, xmax, ymin, ymax):
    if (xmax - xmin) > (ymax - ymin):
        mid = (ymax - ymin) / 2
        span = (xmax - xmin) / 2
        return xmin, xmax, mid - span, mid + span
    else:
        mid = (xmax - xmin) / 2
        span = (ymax - ymin) / 2
        return mid - span, mid + span, ymin, ymax


def main(jsonFileName, updateInterval):
    with open(jsonFileName, 'r') as jsonInputFile:
        logData = json.load(jsonInputFile)

    xmin, xmax, ymin, ymax = equalize_axes(*xy_minmax(logData))

    pathRadius = 4
    ims = []
    execPath = [[], []]

    cmapstr = 'Greys'
    bg = plt.cm.get_cmap(cmapstr)(0.0)

    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor=bg)
    gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1])
    ax1 = fig.add_subplot(gs[0], adjustable='box', aspect=1.0)
    ax1.set_xlim(xmin, xmax)
    ax1.set_ylim(ymin, ymax)
    ax1.grid()
    ax1meshX, ax1meshY = np.meshgrid(np.arange(xmin, xmax, 0.1),
                                     np.arange(ymin, ymax, 0.1))

    ax2 = fig.add_subplot(
        gs[1], adjustable='box', aspect=1.0, facecolor=bg)
    plt.setp(ax2.get_xticklabels(), visible=False)
    plt.setp(ax2.get_yticklabels(), visible=False)
    ax2.set_xlim(-pathRadius, pathRadius)
    ax2.set_ylim(-pathRadius, pathRadius)

    plt.tight_layout(pad=1.08, h_pad=None, w_pad=None, rect=None)

    for stepData in logData:
        x = np.array(stepData['x'])
        y = np.array(stepData['y'])
        ex = np.array(stepData['ex'])
        ey = np.array(stepData['ey'])
        execPath[0].append(x[0])
        execPath[1].append(y[0])

        xRelative = x - x[0]
        yRelative = y - y[0]

        xTracked = x[1:] - ex
        yTracked = y[1:] - ey

        xTrackedRelative = xTracked - x[0]
        yTrackedRelative = yTracked - y[0]

        obstacles = list(map(json_to_obstacle, stepData['obstacles']))
        Phi = np.zeros_like(ax1meshX)
        for each in obstacles:
            Phi += each.phi(ax1meshX, ax1meshY)

        errPath, = ax1.plot(xTracked, yTracked, 'o-', color='grey', lw=4, ms=3)
        path, = ax1.plot(x, y, 'ro-', lw=3, ms=3)
        hist, = ax1.plot(execPath[0], execPath[1], 'b-', lw=2)
        field = ax1.contourf(ax1meshX, ax1meshY, Phi, levels=128, cmap=cmapstr)
        ax1robot, = ax1.plot(x[0], y[0], marker=(3, 0,
                                                 30 + stepData['th'][0] * 180 / np.pi),
                             markersize=20, linestyle='None', color='k')

        obstacleActors = []
        for ob in obstacles:
            if isinstance(ob, OPoint):
                a, = ax1.plot(ob.x, ob.y, 'ro')
                obstacleActors.append(a)

        ax2ErrPath, = ax2.plot(
            xTrackedRelative, yTrackedRelative, 'o-', color='grey', lw=2, ms=5)
        ax2Path, = ax2.plot(xRelative, yRelative, 'ro-', lw=2, ms=5)
        ax2robot, = ax2.plot(0, 0, marker=(3, 0,
                                           30 + stepData['th'][0]*180/np.pi),
                             markersize=20, linestyle='None', color='k')

        ims.append([path, errPath, hist, ax1robot]
                   + obstacleActors
                   + field.collections
                   + [ax2Path, ax2ErrPath, ax2robot]
                   )

    ani = animation.ArtistAnimation(fig, ims, interval=updateInterval)
    plt.show()
    # ani.save('two_moving_obs.mp4',writer='ffmpeg')


parser = argparse.ArgumentParser(
    description='Animate a plot of the NMPC calculation')
parser.add_argument(
    '-f',
    '--file',
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

if __name__ == "__main__":
    main(args.inputFileName, args.interval)
