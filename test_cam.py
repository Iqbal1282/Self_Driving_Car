#!/usr/bin/env python3
"""
Quick camera health-check for Raspberry Pi OS 64-bit (Bookworm)
Works with:  ov5647  imx219  imx708  (Pi-Cam v1/v2/v3 and ArduCam-ov5647)
"""
import time, os
from picamera2 import Picamera2
from libcamera import controls

print("🔍  Camera detection …")
picam2 = Picamera2()
if not picam2.camera_properties:
    print("❌  No camera detected on CSI port")
    exit(1)

print("✅  Found:", picam2.camera_properties['Model'])
print("    Available controls:", ', '.join(picam2.camera_controls.keys()))

# ---- configure ----
config = picam2.create_still_configuration(
    main={"size": (2592, 1944)},          # full sensor
    lores={"size": (320, 240)},           # CNN size
    display="lores"
)

picam2.configure(config)

# ---- optional tweaks ----
picam2.set_controls({"AeEnable": True, "AwbEnable": True})
if "LensPosition" in picam2.camera_controls:
    picam2.set_controls({"LensPosition": 2.5})   # ~1 m focus for M12 lens

picam2.start()

print("\n📷  Live preview (5 s) …  adjust lens if needed")
picam2.start_show_preview()
time.sleep(5)
picam2.stop_show_preview()

print("\n📸  Capturing images …")
req = picam2.capture_request()
req.save("main", "test_full.jpg")
req.save("lores", "test_cnn.jpg")
metadata = req.get_metadata()
req.release()
picam2.stop()

print("\n✅  Saved:")
print("   test_full.jpg  (2592×1944)")
print("   test_cnn.jpg   (320×240)")
print("\nMetadata snapshot:")
for k in ["ExposureTime", "AnalogueGain", "ColourTemperature", "LensPosition"]:
    if k in metadata:
        print(f"   {k}: {metadata[k]}")

print("\n🎉  Camera is alive – you can now run the autopilot.")