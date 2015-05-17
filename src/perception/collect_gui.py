from Tkinter import *
import re
import glob
from fuzzywuzzy import process
from PIL import Image, ImageTk, ImageEnhance
import time
import os
import h5py

from camera import Carmine
import cyni

#light above on in 391 Cory
CARMINE_EXPOSURE=80

# light above off in 391 Cory
#CARMINE_EXPOSURE=40

# SDH Kitchen (light on)
#CARMINE_EXPOSURE=80

# ryan's living room table (2pm)
#CARMINE_EXPOSURE=60

#ryan's shelf
#CARMINE_EXPOSURE=40


stub_cameras = False

def touch(fname, times=None):
    with open(fname, 'a'):
        os.utime(fname, times)

objects = [
'3m_high_tack_spray_adhesive',
]

image_path = "objects"

def load_objects():
    images = glob.glob(image_path + "/*.jpg")
    objects = [x.split("/")[-1].split(".")[0] for x in images]
    return objects

objects = load_objects()

class AutocompleteEntry(Entry):
    def __init__(self, objects, highlight_callback, selection_callback, *args, **kwargs):
        Entry.__init__(self, *args, **kwargs)
        self.objects = objects
        self.var = self["textvariable"]
        if self.var == '':
            self.var = self["textvariable"] = StringVar()

        self.var.trace('w', self.changed)
        self.bind("<Right>", self.selection)
        self.bind("<Return>", self.selection)
        self.bind("<Up>", self.up)
        self.bind("<Down>", self.down)
        self.selection_callback = selection_callback
        self.highlight_callback = highlight_callback

        self.lb = None

    def init_listbox(self):
        # Clear the listbox if it exists, otherwise create
        if self.lb:
            self.lb.delete(0, END)
            return
        self.lb = Listbox(width=self["width"])
        self.lb.bind("<Double-Button-1>", self.selection)
        self.lb.bind("<Return>", self.selection)
        self.lb.place(x=self.winfo_x(), y=self.winfo_y()+self.winfo_height())
        self.selected_index = None
        self.wc = None

    def destroy_listbox(self):
        if self.lb:
            self.lb.destroy()
            self.lb = None
            self.highlight_callback(None)

    def changed(self, name, index, mode):
        # Hide listbox if no text entered
        if self.var.get() == '':
            self.destroy_listbox()
            return

        words = self.comparison()

        if not words:
            self.destroy_listbox()
            self.wc = None
            return

        self.init_listbox()

        for w in words:
            self.lb.insert(END,w)
        self.wc = len(words)

        self.select_index(0)

    def selection(self, event):
        if self.lb:
            item = self.lb.get(ACTIVE).split(' ')[0]
            self.selection_callback(item)
            self.var.set("")
            self.destroy_listbox()
            self.icursor(END)
        return 'break'

    def select_index(self, index):
        if self.selected_index is not None:
            self.lb.selection_clear(self.selected_index)
        self.selected_index = index
        self.lb.selection_set(self.selected_index)
        self.lb.activate(self.selected_index)
        self.lb.see(self.selected_index)
        item = self.lb.get(ACTIVE).split(' ')[0]
        self.highlight_callback(item)

    def get_index(self):
        if self.lb.curselection() == ():
            return 0
        else:
            return int(self.lb.curselection()[0])

    def up(self, event):
        if self.lb:
            index = self.get_index() - 1
            if index == -1:
                index = self.wc-1
            self.select_index(index)

        return 'break'

    def down(self, event):
        if self.lb:
            index = self.get_index() + 1
            if index == self.wc:
                index = 0
            self.select_index(index)

        return 'break'

    def comparison(self):
        results = process.extractBests(self.var.get(), self.objects, score_cutoff=30, limit=10)
        return ["%s (score: %d)" % (s,d) for s,d in results]
        pattern = re.compile('.*' + self.var.get() + '.*')
        return [w for w in self.objects if re.match(pattern, w)]

def selectall(event):
    event.widget.selection_range(0, END)

class App(object):

    def __init__(self, objects, output_dir):
        self.objects = objects
        self.output_dir = output_dir

        self.initialize()
        self.initialize_gui()
        self.initialize_cameras()

        self.images = {}
        for object in objects:
            path = image_path + "/%s.jpg" % object
            if os.path.exists(path):
                image = Image.open(path).resize((200, 140),Image.ANTIALIAS)
                enhancer = ImageEnhance.Brightness(image)
                image = enhancer.enhance(1.25)
                self.images[object] = ImageTk.PhotoImage(image)
            else:
                self.images[object] = None

    def initialize(self):
        self.next_scene = 0
        self.state = 'waiting_on_test_set_name'

    def initialize_gui(self):

        self.root = Tk()
        self.root.bind_class("Entry","<Command-a>", selectall)

        self.test_set_frame = Frame(self.root, padx=5, pady=5)
        self.test_set_frame.grid(row=0, column=0, columnspan=2, sticky=W)
        l = Label(self.test_set_frame, text="Enter test set name")
        l.grid(row=0, column=0)
        self.test_set_name_entry = Entry(self.test_set_frame, width=25)
        self.test_set_name_entry.grid(row=0, column=1)
        self.set_test_set_name = Button(self.test_set_frame, text='Start new test set', command=self.set_test_set)
        self.set_test_set_name.grid(row=0, column=2)
        self.calibrate_button = Button(self.test_set_frame, text='Calibrate', command=self.calibrate)
        self.calibrate_button.grid(row=0, column=3)
        self.calibrate_button.config(state='disabled')

        l = Label(self.root, text="Objects in scene")
        l.grid(row=1, column=0)

        self.lb_objects_in_scene = Listbox(self.root, height=23, width=40)
        self.lb_objects_in_scene.bind("<BackSpace>", self.remove_object)
        self.lb_objects_in_scene.grid(row=2, column=0, rowspan=2)

        self.clear_button = Button(text='clear', command=self.clear_objects)
        self.clear_button.grid(row=4, column=0)
        self.capture_button = Button(text='capture', command=self.capture)
        self.capture_button.config(state='disabled')
        self.capture_button.grid(row=5, column=0, columnspan=2)

        l = Label(self.root, text="Enter objects")
        l.grid(row=1, column=1)

        self.image_label = Label(self.root, height=12)
        self.image_label.grid(row=3, column=1, sticky=S)

        self.object_entry = AutocompleteEntry(objects,
                                              self.show_image,
                                              self.object_selected,
                                              self.root,
                                              width=40)
        self.object_entry.grid(row=2, column=1, sticky=N)

        self.previous_image_frame = Frame(self.root)
        self.previous_image_frame.grid(row=6, column=0, columnspan=2)
        self.last_highres_label = Label(self.previous_image_frame, height=12)
        self.last_highres_label.grid(row=0, column=0)
        self.last_rgbd_label = Label(self.previous_image_frame, height=12)
        self.last_rgbd_label.grid(row=0, column=1)
        self.last_depth_label = Label(self.previous_image_frame, height=12)
        self.last_depth_label.grid(row=0, column=2)
        Label(self.previous_image_frame, text="Canon").grid(row=1, column=0)
        Label(self.previous_image_frame, text="Carmine").grid(row=1, column=1)
        Label(self.previous_image_frame, text="Depth").grid(row=1, column=2)

    def initialize_cameras(self):
        if stub_cameras:
            print "WARNING: Stubbing out cameras; can't actually take pictures."
            return

        settings = {"exposure": CARMINE_EXPOSURE,
                    "gain": 100}
        cyni.initialize()
        device = cyni.getAnyDevice()
        if device is None:
            print "No carmine found."
            print "Please restart the program with the Carmine plugged in."
            self.set_test_set_name.config(state='disabled')
        else:
            self.carmine = Carmine(device, settings)
            self.carmine.initialize()
            self.carmine.start_color_stream()
            self.carmine.start_depth_stream()

    def clear_objects(self):
        self.lb_objects_in_scene.delete(0, END)

    def shutter_stub(self, highres_filename, color_filename, depth_filename):
        touch(depth_filename)
        touch(color_filename)
        touch(highres_filename)

    def set_test_set(self):
        name = self.test_set_name_entry.get()
        path_base = os.path.join(self.output_dir, name)
        if os.path.exists(path_base):
            print "Test set with name %s already exists." % name
            return
        self.test_set_name = name
        os.makedirs(path_base)
        self.move_to_calibration_state()
        self.move_to_collect_state()

    def move_to_calibration_state(self):
        self.set_test_set_name.config(state='disabled')
        self.calibrate_button.config(state='normal')
        self.state = 'start_calibration'

    def move_to_collect_state(self):
        self.calibrate_button.config(text='Done',state='disabled')
        self.capture_button.config(state='normal')
        self.state = 'collecting'

    def calibrate(self):
        path_base = os.path.join(self.output_dir, self.test_set_name)
        if self.state == 'start_calibration':
            self.calibration_path = os.path.join(self.output_dir, 
                                                 self.test_set_name,
                                                 "calibration")
            os.makedirs(self.calibration_path)
            self.calib_scene_num = 0
            self.state = 'calibrating'
            self.calibrate_button.config(text='%d left' % (NUM_CALIBRATION_IMAGES-self.calib_scene_num))
        if self.state == 'calibrating':
            highres_filename = os.path.join(self.calibration_path, 
                                            "NT_{0}.jpg".format(self.calib_scene_num))
            depth_filename = os.path.join(self.calibration_path, 
                                            "NPT_{0}.h5".format(self.calib_scene_num))
            color_filename = os.path.join(self.calibration_path, 
                                            "NPT_{0}.jpg".format(self.calib_scene_num))
            self.shutter(highres_filename, color_filename, depth_filename)
            self.calib_scene_num += 1
            self.calibrate_button.config(text='%d left' % (NUM_CALIBRATION_IMAGES-self.calib_scene_num))
            if self.calib_scene_num == NUM_CALIBRATION_IMAGES:
                self.move_to_collect_state()

    def shutter(self, highres_filename, color_filename, depth_filename):
        if stub_cameras:
            self.shutter_stub(highres_filename, color_filename, depth_filename)
            return
        self.carmine.capture_depth(depth_filename)
        self.carmine.stop_depth_stream()
        print "Took depth"
        self.carmine.capture_color(color_filename)
        self.carmine.start_depth_stream()
        print "Took carmine color"

        rgb = Image.open(color_filename).resize((200, 140),
                                                Image.ANTIALIAS)
        self.rgbd_tk = ImageTk.PhotoImage(rgb)
        with h5py.File(depth_filename, 'r') as df:
            depth_map = cyni.depthMapToImage(df["depth"][:])
            depth_img = Image.fromarray(depth_map).resize((200, 140), 
                                                          Image.ANTIALIAS)
            self.depth_tk = ImageTk.PhotoImage(depth_img)

        self.last_highres_label.config(height=200, image=self.rgbd_tk)
        self.last_rgbd_label.config(height=200, image=self.rgbd_tk)
        self.last_depth_label.config(height=200, image=self.depth_tk)

    def capture(self):
        path_base = os.path.join(self.output_dir, self.test_set_name, "scene_{0}/".format(self.next_scene))
        os.makedirs(path_base)
        self.next_scene += 1

        in_scene = self.lb_objects_in_scene.get(0, END)
        with open(path_base + "objects.txt", 'w') as f:
            for obj in in_scene:
                f.write("%s\n" % obj)

        highres_filename = path_base + "rgb.jpg"
        rgbd_filename = path_base + "rgbd.jpg"
        depth_filename = path_base + "rgbd.h5"


        self.shutter(highres_filename, rgbd_filename, depth_filename)

        print "Done with %s" % (self.next_scene-1)


    def show_image(self, object):
        if object is None:
            self.image_label.config(height=12, image="")
        else:
            if self.images[object] is not None:
                self.image_label.config(height=200, image=self.images[object])

    def object_selected(self, object):
        self.lb_objects_in_scene.insert(END, object)

    def remove_object(self, event):
        self.lb_objects_in_scene.delete(ACTIVE)

    def run(self):
        self.root.mainloop()



if __name__ == '__main__':

    if stub_cameras:
        output_dir = os.path.expanduser("~/workspace/apc_test_data/test_scenes_stubbed")
    else:
        output_dir = os.path.expanduser("~/workspace/apc_test_data/test_scenes")
    app = App(objects, output_dir)
    app.run()
