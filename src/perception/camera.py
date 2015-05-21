import os
import re
import cyni
from PIL import Image
import h5py
import numpy as np
from time import sleep, time
from scipy.misc import imresize

class Carmine(object):

    def __init__(self, device, settings):
        self.device = device
        if settings is None:
            self.settings = {}
        else:
            self.settings = settings

    def initialize(self):
        self.device.open()

        self.serial = self.get_serial()
        self.name = None

    def destroy_color_stream(self):
        self.color_stream.destroy()

    def destroy_depth_stream(self):
        #self.depth_stream.setEmitterState(on=False)
        self.depth_stream.destroy()

    def destroy_ir_stream(self):
        self.ir_stream.destroy()

    def stop_ir_stream(self):
        self.ir_stream.stop()

    def stop_depth_stream(self):
        #self.depth_stream.setEmitterState(on=False)
        self.depth_stream.stop()

    def start_color_stream(self):
        exposure = self.settings.get('exposure', 1)
        gain = self.settings.get('gain', 100)
        width = self.settings.get('color_width', 1280)
        height = self.settings.get('color_height', 1024)
        fps = self.settings.get('color_fps', 30)
        format = self.settings.get('color_format', 'rgb')

        self.color_stream = self.device.createStream("color",
                                                     width=width,
                                                     height=height,
                                                     fps=fps,
                                                     format=format)
        self.color_stream.setMirroring(False)
        self.color_stream.start()

        self.color_stream.setAutoExposureEnabled(False)
        self.color_stream.setAutoExposureEnabled(True)
        self.color_stream.setExposure(exposure)
        self.color_stream.readFrame()
        self.color_stream.readFrame()
        self.color_stream.readFrame()
        self.color_stream.readFrame()
        self.color_stream.setAutoWhiteBalanceEnabled(False)
        color_frame = self.color_stream.readFrame()
        color_frame = self.color_stream.readFrame()
        self.color_stream.setAutoWhiteBalanceEnabled(True)
        for i in range(10):
            color_frame = self.color_stream.readFrame()


    def start_ir_stream(self):

        width = self.settings.get('ir_width', 1280)
        height = self.settings.get('ir_height', 960)
        fps = self.settings.get('ir_fps', 30)
        format = self.settings.get('ir_format', 'gray16')

        self.ir_stream = self.device.createStream("ir",
                                                  width=width,
                                                  height=height,
                                                  fps=fps,
                                                  format=format)
        self.ir_stream.setMirroring(False)
        self.ir_stream.start()
        #self.ir_stream.setEmitterState(on=False)

    def start_depth_stream(self):
        width = self.settings.get('depth_width', 640)
        height = self.settings.get('depth_height', 480)
        fps = self.settings.get('depth_fps', 30)
        format = self.settings.get('depth_format', 'depth100um')

        self.depth_stream = self.device.createStream("depth",
                                                     width=width,
                                                     height=height,
                                                     fps=fps,
                                                     format=format)
        self.depth_stream.setMirroring(False)
        self.depth_stream.start()
        self.device.setImageRegistrationMode("depth_to_color")
        sleep(1)
        self.device.setImageRegistrationMode("depth_to_color")
        #self.depth_stream.setEmitterState(on=False)

    def enable_emitter(self):
        self.depth_stream.setEmitterState(on=True)

    def disable_emitter(self):
        self.depth_stream.setEmitterState(on=False)

    def capture_depth(self, depth_filename, num_extra=6):
        h5py._errors.silence_errors()
        #print 'Capturing depth {0}'.format(depth_filename)

        if os.path.exists(depth_filename):
            os.remove(depth_filename)

        #self.device.setImageRegistrationMode("off")
        depth_frame = self.depth_stream.readFrame()
        #self.depth_stream.setEmitterState(on=True)
        depth_frame = self.depth_stream.readFrame()
        while np.sum(depth_frame) == 0:
            print "waiting on depth frame"
            depth_frame = self.depth_stream.readFrame()
        # Read one more just to ensure the emitter is totally on
        depth_frame = self.depth_stream.readFrame()
        with h5py.File(depth_filename, 'a') as depth_file:
            depth_file.create_dataset("depth", data=depth_frame.data)
            self.depth_data = depth_frame.data
            for i in range(num_extra):
                depth_frame = self.depth_stream.readFrame()
                depth_file.create_dataset("depth_{0}".format(i), data=depth_frame.data)
        #self.depth_stream.setEmitterState(on=False)

        return depth_filename

    def capture_color(self, color_filename):
        #print 'Capturing color {0}'.format(color_filename)

        if os.path.exists(color_filename):
            os.remove(color_filename)

        #self.color_stream.setExposure(80)
        #self.color_stream.setAutoExposureEnabled(False)
        color_frame = self.color_stream.readFrame()
        color_frame = self.color_stream.readFrame()
        #self.color_stream.setAutoWhiteBalanceEnabled(False)
        color_frame = self.color_stream.readFrame()
        color_frame = self.color_stream.readFrame()

        self.color_data = color_frame.data
        Image.fromarray(color_frame.data).save(color_filename, quality=100)
            
        #self.color_stream.setAutoExposureEnabled(True)
        #self.color_stream.setAutoWhiteBalanceEnabled(True)
        return color_filename

    def save_cloud(self, cloud_filename):
        cloud = cyni.depthMapToPointCloud(self.depth_data, self.depth_stream, imresize(self.color_data, self.depth_data.shape))
        cyni.writePCD(cloud, cloud_filename)

    def capture_ir(self, ir_filename):
        print 'Capturing ir {0}'.format(ir_filename)

        if os.path.exists(ir_filename):
            os.remove(ir_filename)

        ir_frame = self.ir_stream.readFrame()
        while ir_frame == None:
            ir_frame = self.ir_stream.readFrame()
        ir_data = np.minimum(ir_frame.data, 51)*5
        raise Exception(4)
        #cv2.imwrite(ir_filename, ir_data)
        return True

    def get_serial(self):
        return self.device.getSerial()

    def close(self):
        self.device.close()

