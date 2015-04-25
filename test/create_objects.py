__author__ = 'dibyo'

import sys
import os.path
import subprocess

import cyni
import Image

import IPython

OUTPUT_DIRECTORY = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                "../data"))

cyni.initialize()
device = cyni.getAnyDevice()
device.open()

def create_pcd(pcd_name):
    depthStream = device.createStream("depth", fps=30)
    colorStream = device.createStream("color", fps=30)

    depthStream.start()
    colorStream.start()

    depthFrame = depthStream.readFrame()
    colorFrame = colorStream.readFrame()

    # Sync depth frame and color frame by offsetting colorFrame
    offset = -6
    colorData = colorFrame.data
    width = colorData.shape[1]
    if offset is not None and offset < 0:
        colorData[:, :width+offset] = colorData[:, -offset:]

    cloud = cyni.depthMapToPointCloud(depthFrame.data, depthStream, colorFrame.data)
    cyni.writePCD(cloud, pcd_name)


def create_ply(pcd_name, ply_name):
    subprocess.call(['pcl_pcd2ply', pcd_name, ply_name])

if __name__ == '__main__':
    output_name = sys.argv[1] if len(sys.argv) >= 2 else 'output'

    pcd_path = os.path.join(OUTPUT_DIRECTORY,
                            'clouds',
                            '{}.pcd'.format(output_name))
    ply_path = os.path.join(OUTPUT_DIRECTORY,
                            'meshes',
                            '{}.ply'.format(output_name))

    create_pcd(pcd_path)
    create_ply(pcd_path, ply_path)

