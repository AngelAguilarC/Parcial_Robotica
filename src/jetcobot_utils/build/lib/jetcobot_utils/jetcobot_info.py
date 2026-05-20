#!/usr/bin/env python3
# encoding: utf-8

import subprocess


def get_lsb_release_info(key):
    try:
        output = subprocess.check_output(['lsb_release', '-s', key]).strip()
        return output.decode()
    except subprocess.CalledProcessError:
        return None

def is_Jetson_Nano():
    if get_lsb_release_info('-c') == 'bionic':
        return True
    return False

def is_Jetson_Orin():
    if get_lsb_release_info('-c') == 'focal':
        return True
    return False

def board_name():
    if get_lsb_release_info('-c') == 'bionic':
        return "Jetson_Nano"
    if self.get_lsb_release_info('-c') == 'focal':
        return "Jetson_Orin"


if __name__ == '__main__':
    print(board_name())
