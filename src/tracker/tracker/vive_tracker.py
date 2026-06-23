import numpy as np

try:
    import openvr
except ImportError as exc:
    raise ImportError(
        "openvr is required for the tracker package. "
        "Install it with `pip install openvr` (it is already in the robot-stack docker image)."
    ) from exc


def _matrix34_to_numpy(matrix34):
    if hasattr(matrix34, 'm'):
        arr = np.asarray(matrix34.m, dtype=np.float64)
    else:
        arr = np.asarray(matrix34, dtype=np.float64)
    arr = arr.reshape(3, 4)

    T = np.eye(4)
    T[:3, :4] = arr
    return T


def _normalize_string(value):
    if isinstance(value, bytes):
        return value.decode('utf-8')
    return str(value)


class ViveTrackerModule:
    def __init__(self):
        self.vr = openvr.init(openvr.VRApplication_Other)
        self.vrsystem = openvr.VRSystem()
        self.object_names = {
            "Tracking Reference": [],
            "HMD": [],
            "Controller": [],
            "Tracker": [],
        }
        self.devices = {}
        poses = self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,
            openvr.k_unMaxTrackedDeviceCount,
        )
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if poses[i].bDeviceIsConnected:
                self.add_tracked_device(i)

    def __del__(self):
        try:
            openvr.shutdown()
        except Exception:
            pass

    def return_selected_devices(self, device_key=""):
        selected_devices = {}
        for key in self.devices:
            if device_key in key:
                selected_devices[key] = self.devices[key]
        return selected_devices

    def add_tracked_device(self, tracked_device_index):
        i = tracked_device_index
        device_class = self.vr.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_Controller:
            device_name = "controller_" + str(len(self.object_names["Controller"]) + 1)
            self.object_names["Controller"].append(device_name)
            self.devices[device_name] = VrTrackedDevice(self.vr, i, "Controller")
        elif device_class == openvr.TrackedDeviceClass_HMD:
            device_name = "hmd_" + str(len(self.object_names["HMD"]) + 1)
            self.object_names["HMD"].append(device_name)
            self.devices[device_name] = VrTrackedDevice(self.vr, i, "HMD")
        elif device_class == openvr.TrackedDeviceClass_GenericTracker:
            device_name = "tracker_" + str(len(self.object_names["Tracker"]) + 1)
            self.object_names["Tracker"].append(device_name)
            self.devices[device_name] = VrTrackedDevice(self.vr, i, "Tracker")
        elif device_class == openvr.TrackedDeviceClass_TrackingReference:
            device_name = "tracking_reference_" + str(len(self.object_names["Tracking Reference"]) + 1)
            self.object_names["Tracking Reference"].append(device_name)
            self.devices[device_name] = VrTrackedDevice(self.vr, i, "Tracking Reference")

    def print_discovered_objects(self):
        for device_type in self.object_names:
            plural = device_type
            if len(self.object_names[device_type]) != 1:
                plural += "s"
            print("Found " + str(len(self.object_names[device_type])) + " " + plural)
            for device in self.object_names[device_type]:
                print(
                    "  "
                    + device
                    + " ("
                    + self.devices[device].get_serial()
                    + ", "
                    + self.devices[device].get_model()
                    + ")"
                )


class VrTrackedDevice:
    def __init__(self, vr_obj, index, device_class):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj
        self.T = np.eye(4)

    def get_serial(self):
        return _normalize_string(
            self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_SerialNumber_String)
        )

    def get_model(self):
        return _normalize_string(
            self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_ModelNumber_String)
        )

    def get_pose_matrix(self, pose=None):
        if pose is None:
            pose = self.vr.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding,
                0,
                openvr.k_unMaxTrackedDeviceCount,
            )
        if pose[self.index].bPoseIsValid:
            return pose[self.index].mDeviceToAbsoluteTracking
        return None

    def get_T(self, pose=None):
        pose_mat = self.get_pose_matrix(pose)
        if pose_mat is not None:
            self.T = _matrix34_to_numpy(pose_mat)
        return self.T
