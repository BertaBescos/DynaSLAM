#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
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
#  * Neither the name of TUM nor the names of its
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
# the resulting .ply file can be viewed for example with meshlab
# sudo apt-get install meshlab

"""
This script plots a trajectory into an image sequence. 
"""

import argparse
import sys
import os
from associate import *
from evaluate import *
from generate_pointcloud import *
from PIL import Image, ImageDraw

focalLength = 525.0
centerX = 319.5
centerY = 239.5

def point(pose,px,py,pz):
    """
    Project a 3D point into the camera.
    
    Input:
    pose -- camera pose
    px,py,pz -- point in global frame
    
    Output:
    u,v -- pixel coordinates
    
    """
    p = pose.dot(numpy.matrix([[px],[py],[pz],[1]]))
    X = p[0,0]
    Y = p[1,0]
    Z = p[2,0]
    u = X/Z * focalLength + centerX
    v = Y/Z * focalLength + centerY
    return [u,v]
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    This script plots a trajectory into an image sequence. 
    ''')
    parser.add_argument('image_list', help='input image list (format: timestamp filename)')
    parser.add_argument('trajectory_file', help='input trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('out_image', help='file name of the result (format: png)')
    args = parser.parse_args()
    
    image_list = read_file_list(args.image_list)
    pose_list = read_file_list(args.trajectory_file)
    traj = read_trajectory(args.trajectory_file)

    matches = associate(image_list, pose_list,0,0.02)

    stamps = image_list.keys()
    stamps.sort()
    
    matches_dict = dict(matches)
    for stamp in stamps:
        image_file = image_list[stamp][0]
        image = Image.open(image_file)
        print "image stamp: %f"%stamp
        
        if stamp in matches_dict: 
            print "pose stamp: %f"%matches_dict[stamp]
            pose = traj[matches_dict[stamp]]
            
            stamps = traj.keys()
            stamps.sort()
        
            xy = []    
            draw = ImageDraw.Draw(image)
            size = 0.01
            
            for s in stamps:
                p = traj[s]
                rel_pose = numpy.dot(numpy.linalg.inv(pose),p)
                if rel_pose[2,3]<0.01: continue
                u,v = point(rel_pose,0,0,0)
                if u<0 or v<0 or u>640 or v>480: continue
                draw.line(point(rel_pose,0,0,0) + point(rel_pose,size,0,0), fill="#ff0000")
                draw.line(point(rel_pose,0,0,0) + point(rel_pose,0,size,0), fill="#00ff00")
                draw.line(point(rel_pose,0,0,0) + point(rel_pose,0,0,size), fill="#0000ff")
            del draw
            
        image.save(os.path.splitext(args.out_image)[0]+"-%f.png"%stamp)
    
    
