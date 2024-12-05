from pypylon import pylon

tl_factory = pylon.TlFactory.GetInstance()
devices = tl_factory.EnumerateDevices()
if not devices:
    print("No cameras found.")
else:
    print(f"Found {len(devices)} camera(s).")
