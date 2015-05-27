#!/usr/bin/env python
from __future__ import division

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
from subprocess import check_call, CalledProcessError
import os
from threading import Thread

from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler


#model_path = "/mnt/data_1/amazon_picking_challenge_models"
model_path = "/mnt/data_1/test_models"
model_path = "/home/arjun/odp/code/detector/test_models"
sift_path = os.path.join(model_path, "sift")
color_path = os.path.join(model_path, "color")

depth_map_units_scale_factor = .0001 # 100um to meters

binpath = '%s/code/bin' % TOP_LEVEL_PATH

run_detector = False
run_features = True
run_segment = True


class APCDetector(object):
    def __init__(self):
        self.camera_name = utils.CAMERA_NAME
        self.calib_file = utils.CAMERA_CALIBRATION_FILE
        self.scoring_categories = ["scores_color_match", "scores_desc_match", "scores_nn_pose"]

        self.calib_extracted = 'calib_extracted'
        self.extract_calibration('{}.json'.format(self.calib_extracted))

        self.detector = None
        if run_detector:
            self.detector = Detector(sift_path, color_path, "")

    @staticmethod
    def get_pose_matrix(pose):
        rmat = np.array(pose['rmat']).reshape((3, 3))
        tvec = np.array(pose['tvec'])
        H = np.eye(4)
        H[:3,:3] = rmat
        H[:3, 3] = tvec.T
        return H

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
        cloud_path = '/home/nmishra/apc_models/{0}/textured_meshes/optimized_poisson_textured_mesh.ply'.format(obj_id)
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

    def process_detections(self, cloud_path, feature_path, calib_path, hypothesis_names):
        detections_list = list()
        pcds = glob.glob('segments_*.pcd')
        pcd_ind = 0
        if run_detector:
            for pcd in pcds:
                print 'processing segment %d of %d' % (pcd_ind + 1, len(pcds))
                if pcd.endswith('all.pcd'):
                    check_call(['rm', pcd])
                    check_call(['rm', pcd.replace('.pcd',  '.jpg')])
                    check_call(['rm', pcd.replace('_all.pcd',  '_mask.jpg')])
                else:
                    img = pcd.replace('.pcd', '.jpg')
                    mask = img.replace('.jpg', '_mask.jpg')
                    full_pcd = cloud_path
                    feat = feature_path
                    caminfo = calib_path
                    detection_json = self.detector.process_apc(img, mask, pcd, full_pcd, feat, caminfo, '', '',
                                                   str(','.join(hypothesis_names)))

                    check_call(['rm', pcd])
                    check_call(['rm', pcd.replace('.pcd',  '.jpg')])
                    check_call(['rm', pcd.replace('.pcd',  '_mask.jpg')])

                    detections = json.loads(detection_json)
                    print 'detections', detections
                    if detections['detections'] is not None:
                        for detection in detections['detections']:
                            for score_category in self.scoring_categories:
                                detection[score_category] = \
                                    {obj_id: score for obj_id, score in detection[score_category]}
                            detection['segment'] = pcd

                    detections_list.append(detections)
                    pcd_ind += 1

            check_call(['rm', 'segments_plane.json'])

            json.dump(detections_list, open('detection_out.json', 'w'))
        else:
            detections_list = json.load(open('detection_out.json', 'r'))
        return detections_list

    @staticmethod
    def viz_detections(img_path, obj_ids, poses, K):
        img = imread(img_path)
        for k in xrange(len(obj_ids)):
            obj_id = obj_ids[k]
            pose = poses[k]
            H = np.linalg.inv(APCDetector.get_pose_matrix(pose))
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
                     calibration_path, transform_path, cubbyhole, hypothesis_names):
        calibration = h5.File(calibration_path)
        rgb_K, depth_K, H_rgb_from_depth, depth_scale = self.load_calibration(calibration)

        self.make_cloud(depth_map_path, image_path, cloud_path, rgb_K, depth_K, H_rgb_from_depth, depth_scale)

        if run_segment:
            self.segment(cloud_path, image_path, segment_base, calibration_path, transform_path, cubbyhole)
        if run_features:
            self.extract_features(image_path, "features.h5", "segments_mask.jpg")

        data = self.process_detections(cloud_path, "features.h5", calibration_path.replace("h5", "json"),
                                       hypothesis_names)

        def calculate_score(segment, object_id, all_object_ids):
            total_score = 0

            for scoring_category in self.scoring_categories:
                score_sum = 0
                object_score = 0
                for id_ in all_object_ids:
                    # print 'score', segment[scoring_category][id_]
                    score_sum += segment[scoring_category][id_]
                    if id_ == object_id:
                        object_score = segment[scoring_category][id_]

                # print 'totals', object_score, score_sum
                if score_sum:
                    total_score += object_score / score_sum

            return total_score / segment["cluster_size"]

        def assign_object_ids(segments, hypothesis_names):
            detections = []

            for segment in segments:
                if segment['detections']:
                    for d in segment['detections']:
                        detections.append(d)

            undetected_objects = set(hypothesis_names)
            # detected_objects = []
            object_poses = {}
            undetected_segments = list()

            for d in detections:
                obj_id = d['object_id']
                if not obj_id:
                    undetected_segments.append(d)
                else:
                    print 'Detected {}'.format(obj_id)
                    object_poses[obj_id] = d['zz_pose_map'][obj_id]
                    undetected_objects.remove(obj_id)

            for hypothesis in hypothesis_names:
                if hypothesis not in object_poses:
                    if undetected_segments:
                        # Choose from remaining segments
                        undetected_segment_scores = []

                        for segment in undetected_segments:
                            score = calculate_score(segment, hypothesis, hypothesis_names)
                            undetected_segment_scores.append(score)
                        detected_segment_index = np.argmax(undetected_segment_scores)

                        print 'Detected {}'.format(hypothesis)
                        object_poses[hypothesis] = \
                            undetected_segments[detected_segment_index]["zz_pose_map"][hypothesis]
                        # undetected_segments.remove(detected_segment)
                        undetected_objects.remove(hypothesis)

            # obj_ids, poses = zip(object_poses.items())
            # obj_ids = []
            # poses = []
            # for obj_id, pose in object_poses.iteritems():
            #     obj_ids.append(obj_id)
            #     poses.append(pose)
            # viz_img = APCDetector.viz_detections(image_path, obj_ids, poses, rgb_K)
            # imsave('detections.png', viz_img)

            return object_poses

        return assign_object_ids(data, hypothesis_names)

    def extract_calibration(self, output_path):
        print self.calib_file
        calibration = h5.File(self.calib_file)
        rgb_name = "{0}_rgb_K".format(self.camera_name)
        depth_name = "{0}_depth_K".format(self.camera_name)
        H_rgb_name = "H_{0}_from_{0}".format(self.camera_name)  # "H_%s_from_PS" % camera_name
        H_depth_name = "H_{0}_ir_from_{0}".format(self.camera_name) # "H_%s_ir_from_PS" % camera_name
        depth_scale_name = "{0}_depth_scale".format(self.camera_name)
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

    def run_pipeline(self, image_file, depth_file, transform_file, cubbyhole, hypothesis_names):
        object_poses = self.detect_scene(image_file, depth_file, 'cloud.pcd', 'segments', 'detections',
                                           '{}.h5'.format(self.calib_extracted), transform_file, cubbyhole,
                                           hypothesis_names)

        object_poses = {object_id: self.get_pose_matrix(pose)
                        for object_id, pose in object_poses.iteritems()}

        # Remove data
        check_call(['rm',
                    # 'cloud.pcd',
                    'features.h5',
                    'detection_out.json'])

        return object_poses


class NewCubbyholeRequestHandler(PatternMatchingEventHandler):
    """
    example_json_request = {
        "bin_name": "bin_G",
        'contents': ['dove_beauty_bar', 'kong_duck_dog_toy']
    }
    """
    patterns = ['*.json']

    def __init__(self, *args, **kwargs):
        super(NewCubbyholeRequestHandler, self).__init__()

        self.object_poses_directory = utils.OBJECT_POSES_DIR
        self.ssh_object_poses_directory = None
        if utils.COMPUTER != utils.ROS_COMPUTER:
            self.ssh_object_poses_directory = '{}:{}'.format(utils.ROS_COMPUTER,
                                                             self.object_poses_directory)

        self.apc_detector = APCDetector(*args, **kwargs)
        self.thread = None

    def __enter__(self, *args, **kwargs):
        self.observer = Observer()
        self.observer.schedule(self, path=utils.PERCEPTION_REQUEST_DIR)
        self.observer.start()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.observer.stop()
        self.observer.join()

    def process_cubbyhole(self, event):
        print 'received new request in {}'.format(event.src_path)
        try:
            with open(event.src_path, 'r') as f:
                cubbyhole = json.load(f)
        except IOError:
            return

        scene_directory = os.path.join(utils.PERCEPTION_DIR, cubbyhole['bin_name'])
        image_file = os.path.join(scene_directory, 'rgbd.jpg')
        depth_file = os.path.join(scene_directory, 'rgbd.h5')
        transform_file = os.path.join(scene_directory, 'transform.txt')
        pose_fix_directory = os.path.join(utils.POSE_FIX_DIR)

        try:
            hypothesis_names = cubbyhole['objects']
            cubbyhole_name = cubbyhole['bin_name']
        except KeyError as e:
            print "Request does not contain the required information"
            print e
            return

        try:
            cubbyhole_dims = utils.bin_dims[cubbyhole_name]
        except KeyError as e:
            print "Request contains the wrong bin"
            return

        try:
            object_poses = self.apc_detector.run_pipeline(image_file, depth_file, transform_file,
                                                          cubbyhole_dims, hypothesis_names)
        except CalledProcessError as e:
            print "Error in C++ code"
            print e
            return

        transform = np.loadtxt(transform_file)
        print 'transform', transform
        object_poses_json = {}
        for obj_id, pose in object_poses.iteritems():
            pose_fix_transform = np.loadtxt(os.path.join(pose_fix_directory, '{}_pose.csv'.format(obj_id)),
                                            delimiter=',')
            pose_fixed = np.linalg.inv(np.array(pose)) .dot(np.linalg.inv(pose_fix_transform))
            transformed_pose = transform.dot(pose_fixed)
            object_poses_json[obj_id] = transformed_pose.tolist()
        object_poses_file = os.path.join(self.object_poses_directory,
                                         os.path.basename(event.src_path))
        with open(object_poses_file, 'w') as f:
            json.dump({
                'bin_name': cubbyhole_name,
                'object_poses': object_poses_json
            }, f)
        if self.ssh_object_poses_directory is not None:
            check_call(['scp', object_poses_file, self.ssh_object_poses_directory])

    def on_modified(self, event):
        while self.thread is not None:
            sleep(1)

        self.thread = Thread(target=self.process_cubbyhole, args=(event,))
        self.thread.start()
        self.thread.join()
        self.thread = None


if __name__ == '__main__':
    with NewCubbyholeRequestHandler() as handler:
        print 'ready to execute perception'
        while True:
            sleep(1)



