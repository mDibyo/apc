__author__ = 'dibyo'

from detectorpy import Detector
from pipeline_config import TOD_DATA_SUBDIRS, TOP_LEVEL_PATH

import glob
import cycloud
from scipy.misc import imread, imsave
import h5py as h5
import numpy as np

import utils
from time import sleep
import json
from subprocess import check_call
import os
from argparse import ArgumentParser

from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler


#model_path = "/mnt/data_1/amazon_picking_challenge_models"
model_path = "/mnt/data_1/test_models"
model_path = "/home/arjun/odp/code/detector/test_models"
sift_path = os.path.join(model_path, "sift")
color_path = os.path.join(model_path, "color")

depth_map_units_scale_factor = .0001 # 100um to meters

binpath = '%s/code/bin' % TOP_LEVEL_PATH

run_detector = True
run_features = True
run_segment = True


class APCDetector(object):
    def __init__(self, camera_name, calib_file):
        self.detector = None
        if run_detector:
            self.detector = Detector(sift_path, color_path, "")

        self.camera_name = camera_name,
        self.calib_file = calib_file,

    @staticmethod
    def project_cloud_to_image(img, cloud, K):
        pts3 = cloud[0, :, :3]
        img_pts = cycloud.projectPoints(K, pts3)
        img_dim = img.shape[0:2]

        z_buffer = np.zeros(img_dim, dtype=np.int8) - 1
        idx_buffer = np.zeros(img_dim, dtype=np.int64) - 1
        proj_img = np.copy(img)

        for k in xrange(img_pts.shape[0]):
            pt = pts3[k, :]
            px = img_pts[k, :]

            r = px[1]
            c = px[0]

            if r < 0 or c < 0 or r >= img_dim[0] or c >= img_dim[1]:
                continue
            if z_buffer[r, c] > 0 and pt[2] > z_buffer[r, c]:
                continue

            z_buffer[r, c] = pt[2]
            idx_buffer[r, c] = k
            proj_img[r, c, :] = cloud[0, k, 3:]

        idx_buffer = np.ma.masked_less(idx_buffer, 0)
        vis_inds = idx_buffer[~idx_buffer.mask]

        return proj_img, vis_inds

    @staticmethod
    def get_obj_cloud(obj_id):
        cloud_path = '/mnt/teebee/amazon_picking_challenge/{0}/textured_meshes/optimized_poisson_textured_mesh.ply'.format(obj_id)
        return cycloud.readPLY(cloud_path).cloud

    @staticmethod
    def make_cloud(depth_map_path, image_path, output_path, rgb_K, depth_K, H_rgb_from_depth, depth_scale):

        with h5.File(depth_map_path) as depth_map_file:
            depth_map = depth_map_file["depth"].value

        image = imread(image_path)

        unregistered = depth_map * depth_map_units_scale_factor * depth_scale
        window = 21
        sigma_depth = 0.00635 # in meters (0.25 inches) **UNITS**
        sigma_pixels = 10.5
        filtered = cycloud.bilateral_filter(unregistered, window, sigma_depth, sigma_pixels)
        registered = cycloud.registerDepthMap(filtered, image, depth_K, rgb_K, H_rgb_from_depth)

        cloud = cycloud.registeredDepthMapToPointCloud(registered, image, rgb_K)
        cycloud.writePCD(cloud, output_path)

    @staticmethod
    def segment(cloud_path, image_path, output_base, calibration_path, transform_path, cubbyhole):
        cmd = ('{binpath}/segment_shelf_apc --pcd {pcd} --image {img} --calibration {calib}' +
               ' --rootname {output_base} --config_file {cfg} --nth_highest_plane {nhp}' +
               ' --transform_file {transform} --cubbyhole_front {cubbyhole_front}' +
               ' --cubbyhole_back {cubbyhole_back} --cubbyhole_left {cubbyhole_left}' +
               ' --cubbyhole_right {cubbyhole_right} --cubbyhole_bottom {cubbyhole_bottom}' +
               ' --cubbyhole_top {cubbyhole_top}').format(
            binpath=binpath,
            output_base=output_base,
            pcd=cloud_path,
            img=image_path,
            calib=calibration_path,
            cfg='%s/pipeline/config/empty.cfg' % TOP_LEVEL_PATH,
            nhp=1,
            transform=transform_path,
            cubbyhole_front=cubbyhole[0],
            cubbyhole_back=cubbyhole[1],
            cubbyhole_left=cubbyhole[2],
            cubbyhole_right=cubbyhole[3],
            cubbyhole_bottom=cubbyhole[4],
            cubbyhole_top=cubbyhole[5])

        print cmd
        check_call(cmd, shell=True)

    @staticmethod
    def extract_features(image_path, output_path, mask_path):
        cmd = 'python {featpath}/extract_features.py {img} --mask {mask} dsift {out} --invert_mask'.format(
            featpath='%s/features' % TOP_LEVEL_PATH,
            mask=mask_path,
            img=image_path,
            out=output_path)
        print cmd
        check_call(cmd, shell=True)

    def process_detections(self, cloud_path, feature_path, calib_path):
        djs = list()
        pcds = glob.glob('segments_*.pcd')
        pcd_ind = 0
        if run_detector:
            for pcd in pcds:
                print 'processing segment %d of %d' % (pcd_ind + 1, len(pcds))
                if pcd.endswith('all.pcd'):
                    continue
                img = pcd.replace('.pcd', '.jpg')
                mask = img.replace('.jpg', '_mask.jpg')
                full_pcd = cloud_path
                feat = feature_path
                caminfo = calib_path
                js = self.detector.process(img, mask, pcd, full_pcd, feat, caminfo, '', '', '')

                djs.append(json.loads(js))
                pcd_ind += 1
            json.dump(djs, open('detection_out.json', 'w'))
        else:
            djs = json.load(open('detection_out.json', 'r'))
        return djs

    @staticmethod
    def viz_detections(img_path, obj_ids, poses, K):
        img = imread(img_path)
        for k in xrange(len(obj_ids)):
            obj_id = obj_ids[k]
            pose = poses[k]
            rmat = np.array(pose['rmat']).reshape((3, 3))
            tvec = np.array(pose['tvec'])
            H = np.eye(4)
            H[:3,:3] = rmat
            H[:3, 3] = tvec.T
            H = np.linalg.inv(H)
            cloud = cycloud.transformCloud(APCDetector.get_obj_cloud(obj_id), H)
            print img.shape
            img, _ = APCDetector.project_cloud_to_image(img, cloud, K)
        return img

    @staticmethod
    def load_calibration(calibration):
        rgb_K = calibration["rgb_K"].value
        depth_K = calibration["depth_K"].value
        H_rgb_from_depth = calibration["H_rgb_from_depth"].value
        depth_scale = calibration["depth_scale"].value
        return rgb_K, depth_K, H_rgb_from_depth, depth_scale

    def detect_scene(self, image_path, depth_map_path, cloud_path, segment_base, detections_path,
                     calibration_path, transform_path, cubbyhole):
        calibration = h5.File(calibration_path)
        rgb_K, depth_K, H_rgb_from_depth, depth_scale = self.load_calibration(calibration)

        self.make_cloud(depth_map_path, image_path, cloud_path, rgb_K, depth_K, H_rgb_from_depth, depth_scale)

        if run_segment:
            self.segment(cloud_path, image_path, segment_base, calibration_path, transform_path, cubbyhole)
        if run_features:
            self.extract_features(image_path, "features.h5", "segments_mask.jpg")

        data = self.process_detections(cloud_path, "features.h5", calibration_path.replace("h5", "json"))

        detections = []

        for dl in data:
            if not dl['detections']:
                continue
            for d in dl['detections']:
                detections.append(d)

        obj_ids = list()
        poses = list()
        for d in detections:
            obj_id = d['object_id']

            if obj_id == '':
                continue
            print d
            print 'Detected %s' % obj_id
            obj_ids.append(obj_id)
            poses.append(d['zz_pose_map'][obj_id])

        viz_img = APCDetector.viz_detections(image_path, obj_ids, poses, rgb_K)
        imsave('detections.png', viz_img)

    def extract_calibration(self, output_path):
        calibration = h5.File(self.calib_file)
        rgb_name = "%{0}_rgb_K".format(self.camera_name)
        depth_name = "%{0}_depth_K".format(self.camera_name)
        H_rgb_name = "H_{0}_from_{0}".format(self.camera_name)  # "H_%s_from_PS" % camera_name
        H_depth_name = "H_{0}_ir_from_{0}".format(self.camera_name) # "H_%s_ir_from_PS" % camera_name
        depth_scale_name = "%{0}_depth_scale".format(self.camera_name)
        rgb_K = calibration[rgb_name].value
        depth_K = calibration[depth_name].value
        H_rgb_from_ref = calibration[H_rgb_name].value
        H_ir_from_ref = calibration[H_depth_name].value
        H_rgb_from_depth = np.dot(H_rgb_from_ref, np.linalg.inv(H_ir_from_ref))
        depth_scale = calibration[depth_scale_name].value

        output_h5_path = output_path.replace('json', 'h5')
        if os.path.exists(output_h5_path):
            os.remove(output_h5_path)
        with h5.File(output_h5_path, 'w') as out:
            out["rgb_K"] = rgb_K
            out["depth_K"] = depth_K
            out["H_rgb_from_depth"] = H_rgb_from_depth
            out["depth_scale"] = depth_scale

        P = np.zeros((4, 4))
        P[:3, :3] = rgb_K
        P[3,2] = 1.0

        info = {"D": [0.0, 0.0, 0.0, 0.0, 0.0],
                "frame_id": "/openni_rgb_optical_frame",
                "K": rgb_K.flatten().tolist(),
                "height": 1024,
                "width": 1280,
                "P": P.flatten().tolist(),
                "distortion_model": "plumb_bob",
                "R": np.eye(3).flatten().tolist()
        }
        with open(output_path, 'w') as outfile:
            json.dump(info, outfile)

    def run_pipeline(self, image_file, depth_file, transform_file, cubbyhole):
        calib_extracted = 'calib_extracted'
        self.extract_calibration('{}.json'.format(calib_extracted))

        self.detect_scene(image_file, depth_file, 'cloud.pcd', 'segments', 'detections',
                          '{}.h5'.format(calib_extracted), transform_file, cubbyhole)



class NewCubbyholeRequestHandler(PatternMatchingEventHandler):
    """
    example_json_request = {
        "cubbyhole": "bin_G"
    }
    """
    patterns = ['*.json']

    def __init__(self, *args, **kwargs):
        super(NewCubbyholeRequestHandler, self).__init__()

        self.apc_detector = APCDetector(*args, **kwargs)

    def __enter__(self, *args, **kwargs):
        self.observer = Observer()
        self.observer.schedule(NewCubbyholeRequestHandler(*args, **kwargs),
                          path=utils.PERCEPTION_REQUEST_DIR)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.observer.stop()
        self.observer.join()

    def process_cubbyhole(self, event):
        print 'received new request in '.format(event.src_path)
        with open(event.src_path, 'r') as f:
            cubbyhole = json.load(f)

            scene_directory = os.path.join(utils.PERCEPTION_DIR, cubbyhole['bin_name'])
            image_file = os.path.join(scene_directory, 'rgbd.jpg')
            depth_file = os.path.join(scene_directory, 'rgbd.h5')
            transform_file = os.path.join(scene_directory, 'transform.txt')

            cubbyhole = utils.CUBBYHOLE_LENS

            self.apc_detector.run_pipeline(image_file, depth_file, transform_file, cubbyhole)

    def on_modified(self, event):
        self.process_cubbyhole(event)


if __name__ == '__main__':
    inch_to_m = 0.0254

    parser = ArgumentParser()
    parser.add_argument('-n', '--camera-name')
    parser.add_argument('-c', '--calib-file')
    parser.add_argument('--cubbyhole', type=json.loads)
    parser.add_argument('--perception-request-dir', )
    args = parser.parse_known_args()[0]

    cubbyhole = args.cubbyhole
    # cubbyhole = [-18, 0, -6, 6, 37, 47] # [front, back, left, right, bottom, top]
    cubbyhole = [x * inch_to_m for x in cubbyhole]

    with NewCubbyholeRequestHandler(camera_name=args.camera_name,
                                    calib_file=args.calib_file) as handler:
        while True:
            print 'ready to execute perception'
            sleep(0)