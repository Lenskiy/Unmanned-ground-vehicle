#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Boris Gromov, BioRobotics Lab at Korea Tech
# All rights reserved.
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# Some parts of this code were adopted from ros_comm/rostopic sources
# of Robot Operating System (ROS).
#

# v0.1

# TODO for v0.2:
#  - Load config from file
#  - Aliases
#  - Numerical expressions

import sys
import yaml
from select import select
from select import error as select_error

def get_val(d, p):
    """
    Retrieve data from structure with given path
    :param d: any combination of dictionaries or lists
    :param p: path to the object
    """
    
    if p[0] == '\"': return p.split('\"')[1::2][0]

    key = p.strip('/')
    rest_key = ''
    val = None
    
    #print 'Input d:', d, 'p:', p
    if not len(key):
        sys.stderr.write('Error: empty path\n')
        return None
    if not len(d):
        sys.stderr.write('Error: empty dictionary\n')
        return None
    
    if '/' in key: key, rest_key = key.split('/', 1)

    # Get index
    idx = None
    if '[' in key:
        idx = key.split('[', 1)[1];
        if ']' in idx:
            idx = idx.split(']')[0]
            try:
                idx = int(idx)
            except ValueError:
                sys.stderr.write('Error: array index should be integer\n')
                return None
            key = key.split('[', 1)[0]

    if idx != None:
        try:
            val = d.get(key)[idx]
        except IndexError:
                sys.stderr.write('Error: array index is out of range\n')
                return None
        except TypeError:
                sys.stderr.write('Error: "%s" is not array\n' % key)
                return None
    else:
        val = d.get(key)

    if rest_key != '' and type(val) == dict:
        return get_val(val, rest_key)
    else:
        if val == None:
            sys.stderr.write('Error: invalid path -- key "%s" does not exist\n' % key)
            return None
        elif type(val) != dict and rest_key != '':
            sys.stderr.write('Error: "%s" is not a dictionary\n' % key)
            return None
        return val
    return None

def path_to_val(cfg, data):
    """
    Recursively crawl through 'cfg' and substitute all paths with values from 'data'
    :param cfg: any combination of arrays and dicitionaries
    :param data: contains data for substitution
    """
    val = None
    if type(cfg) == list:
        val = []
        #print 'List:', cfg
        for i in cfg:
            val.append(path_to_val(i, data))
    elif type(cfg) == dict:
        val = {}
        #print 'Dict:', cfg
        for i in cfg.keys():
            #print 'Key:', i
            val[i] = path_to_val(cfg[i], data)
    else:
        #print 'Value:', cfg
        return get_val(data, cfg)
    return val

def recompose(cfg):
    """
    Recompose message
    :param cfg: object that describes layout of output message
    """
    new_msg = None

    #aliases = cfg['__aliases__']

    for msg in stdin_yaml_arg():
        if msg and cfg:
            new_msg = path_to_val(cfg, msg)
            sys.stdout.write(yaml.dump(new_msg) + '---\n')
            
    return new_msg

def stdin_yaml_arg():
    """
    Iterate over YAML documents in stdin
    :returns: for next list of arguments on stdin. Iterator returns a list of args for each call, ``iterator``
    """
    try:
        arg = 'x'
        rlist = [sys.stdin]
        wlist = xlist = []
        while arg != '\n':
            buff = ''
            while arg != '' and arg.strip() != '---':
                val, _, _ = select(rlist, wlist, xlist, 1.0)
                if not val:
                    continue
                # sys.stdin.readline() returns empty string on EOF
                arg = sys.stdin.readline() 
                if arg != '' and arg.strip() != '---':
                    buff = buff + arg

            if arg.strip() == '---': # End of document
                try:
                    loaded = yaml.load(buff.rstrip())
                except Exception as e:
                    sys.stderr.write("Invalid YAML: %s\n"%str(e))
                if loaded is not None:
                    yield loaded
            elif arg == '': #EOF
                # we don't yield the remaining buffer in this case
                # because we don't want to publish partial inputs
                return
            
            arg = 'x' # reset

    except select_error:
        return # most likely ctrl-c interrupt

if __name__ == '__main__':
    cfg = yaml.load(sys.argv[1])
    try:
        recompose(cfg)
    except KeyboardInterrupt:
        print ''

