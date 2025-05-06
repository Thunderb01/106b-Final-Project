#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
Adapted for Spring 2020 by Amay Saxena
"""
import numpy as np
import sys
import argparse
import rospy

from swarm_explorer.msg import ExplorerState


def parse_args():
    """
    Function for parsing command line arguments.
    """
    parser = argparse.ArgumentParser()
    return parser.parse_args()


def main():
    rospy.init_node("swarm")
    parser = parse_args()


if __name__ == "__main__":
    main()
