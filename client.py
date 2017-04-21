#!/usr/bin/python
import Pyro4
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-d','--direction', help='1 for clockwise, -1 for counter clockwise', required=True)
parser.add_argument('-s','--steps', help='Number of steps', required=True)
parser.add_argument('-w','--delayms', help='Delay in ms', required=True)
args = vars(parser.parse_args())


robot = Pyro4.Proxy("PYRONAME:autox")
print(robot)
steps = str(args['steps'])
direction = str(args['direction'])
delayms = str(args['delayms'])
robot.MoveRotor(steps,direction,delayms)

