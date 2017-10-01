from styx_msgs.msg import TrafficLight
import rospkg
import os
import sys
import tensorflow as tf
import numpy as np
from functools import partial

THRESHOLD = 0.50

class TLClassifierOB(object):
    def __init__(self):
        

        self.tf_session = None
        self.predict = None
        self.clabels = [4, 0, 1, 2, 4, 4]
        self.readsize = 1024

    def get_classification(self, image_np):
        """Determines the color of the traffic light in the image

        Args:
            image_np (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        ros_root = rospkg.get_ros_root()

	r = rospkg.RosPack()
	path = r.get_path('tl_detector')
	print(path)

        # set up tensorflow and traffic light classifier
        if self.tf_session is None:
            # get the traffic light classifier
            self.config = tf.ConfigProto(log_device_placement=True)
            self.config.gpu_options.per_process_gpu_memory_fraction = 0.5  # don't hog all the VRAM!
            self.config.operation_timeout_in_ms = 50000 # terminate anything that don't return in 50 seconds
            self.tf_graph = tf.Graph()
            with self.tf_graph.as_default():
                od_graph_def = tf.GraphDef()
                print('start here 1')
                with tf.gfile.GFile(path+'/frozen_inference_graph.pb', 'rb') as fid:
                    print('read GFile')
                    serialized_graph = fid.read()
                    od_graph_def.ParseFromString(serialized_graph)
                    tf.import_graph_def(od_graph_def, name='')
                    self.tf_session = tf.Session(graph=self.tf_graph, config=self.config)
                    # Definite input and output Tensors for self.tf_graph
                    self.image_tensor = self.tf_graph.get_tensor_by_name('image_tensor:0')
                    # Each score represent how level of confidence for each of the objects.
                    # Score is shown on the result image, together with the class label.
                    self.detection_scores = self.tf_graph.get_tensor_by_name('detection_scores:0')
                    self.detection_classes = self.tf_graph.get_tensor_by_name('detection_classes:0')
                    self.num_detections = self.tf_graph.get_tensor_by_name('num_detections:0')
                    self.predict = True
                    print('Maybed not read frozen inference graph')

        predict = TrafficLight.UNKNOWN
        if self.predict is not None:
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image_np, axis=0)

            # Actual detection
            
            (scores, classes, num) = self.tf_session.run(
                [self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np_expanded})

            # Visualization of the results of a detection.
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)

            # calculate prediction
            c = 5
            predict = self.clabels[c]
            cc = classes[0]
            confidence = scores[0]
            print('cc:', cc)
            print('confidence:', confidence)
            print('predict is:', predict)
            if cc > 0 and cc < 4 and confidence is not None and confidence > THRESHOLD:
                c = cc
                predict = self.clabels[c]
        print('Light:', predict)
        return predict
