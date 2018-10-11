import rospy
from std_msgs.msg import String

# Result for BODY_25 (25 body parts consisting of COCO + foot)
# see more BODY_25: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md#pose-output-format-body_25
POSE_BODY_25_BODY_PARTS = {
    0:  "Nose",
    1:  "Neck",
    2:  "RShoulder",
    3:  "RElbow",
    4:  "RWrist",
    5:  "LShoulder",
    6:  "LElbow",
    7:  "LWrist",
    8:  "MidHip",
    9:  "RHip",
    10: "RKnee",
    11: "RAnkle",
    12: "LHip",
    13: "LKnee",
    14: "LAnkle",
    15: "REye",
    16: "LEye",
    17: "REar",
    18: "LEar",
    19: "LBigToe",
    20: "LSmallToe",
    21: "LHeel",
    22: "RBigToe",
    23: "RSmallToe",
    24: "RHeel",
    25: "Background"
}

POSE_BODY_25_BODY_PARTS_CONV = {name: index for index, name in POSE_BODY_25_BODY_PARTS.items()}

class OPKpsListener:

	def __init__(self):
		rospy.init_node('OPKps_Listener')
		rospy.Subscriber('op_25kps', String, self.lis_func)
		rospy.spin()
	def lis_func(self, data):
		kpts = self._parse_25kps(data.data)
		self._show_25kps(kpts)

		pb = POSE_BODY_25_BODY_PARTS_CONV
		print(kpts[pb['RWrist']][2],  kpts[pb['RShoulder']][2])
		if kpts[pb['RWrist']][2] >1.2* kpts[pb['RShoulder']][2]:
			print('put up right hand')

	def _parse_25kps(self, data):
		kpts = []
		for i, kpt in enumerate(data.split('\n')):
			if i==0 or kpt == '': 
				continue
			x, y, z = kpt.split()
			kpts.append((float(x), float(y), float(z)))
		return kpts

	def _show_25kps(self, kps):
		pb = POSE_BODY_25_BODY_PARTS_CONV
		shows = {pb['Nose'], pb['RShoulder'], pb['RElbow'], pb['RWrist']}
		for i, kpt in enumerate(kps):
			if i not in shows: continue
			print(POSE_BODY_25_BODY_PARTS[i] + ': ' +str(kpt))
		print('\n')

if __name__ == '__main__':
	OPKpsListener()
