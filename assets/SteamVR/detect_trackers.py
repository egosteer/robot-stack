#!/usr/bin/env python3
"""List the SteamVR serial numbers of connected Vive trackers.

Run this with SteamVR already running (./launch.sh, in the container) to read each
tracker's serial number (e.g. LHR-XXXXXXXX), then copy them into the left/right serials
in src/tracker/launch/dual_tracker.launch.py.
"""
import sys
import time

try:
    import openvr
except ImportError:
    sys.exit("openvr is not installed (pip install openvr).")


def main():
    try:
        vr = openvr.init(openvr.VRApplication_Other)
    except openvr.OpenVRError as exc:
        sys.exit(f"Could not connect to SteamVR — is it running (./launch.sh)?\n  {exc}")

    # Give the drivers a moment to report connected devices.
    time.sleep(1.0)

    trackers = []
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        if vr.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_GenericTracker:
            continue
        serial = vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
        model = vr.getStringTrackedDeviceProperty(i, openvr.Prop_ModelNumber_String)
        connected = vr.isTrackedDeviceConnected(i)
        trackers.append((i, serial, model, connected))

    if not trackers:
        print("No Vive trackers detected.")
        print("Check that the trackers are powered on and paired, and that the base")
        print("stations and USB receiver are connected.")
    else:
        print(f"Found {len(trackers)} tracker(s):\n")
        for index, serial, model, connected in trackers:
            state = "connected" if connected else "not connected"
            print(f"  [{index}] {serial}   ({model}, {state})")
        print("\nCopy the serials into src/tracker/launch/dual_tracker.launch.py")
        print("(left_serial_number / right_serial_number).")

    openvr.shutdown()


if __name__ == "__main__":
    main()
