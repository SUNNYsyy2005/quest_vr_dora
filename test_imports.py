#!/usr/bin/env python3
"""Test script to verify all imports work correctly"""

import sys
import os

# Add nodes directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'nodes'))

print("Testing imports...")
print("-" * 40)

# Test core dependencies
try:
    import dora
    print("✅ dora")
except ImportError as e:
    print(f"❌ dora: {e}")

try:
    import pyarrow
    print("✅ pyarrow")
except ImportError as e:
    print(f"❌ pyarrow: {e}")

try:
    import numpy
    print("✅ numpy")
except ImportError as e:
    print(f"❌ numpy: {e}")

# Test VR reader dependencies
try:
    from FPS_counter import FPSCounter
    print("✅ FPS_counter")
except ImportError as e:
    print(f"❌ FPS_counter: {e}")

try:
    from buttons_parser import parse_buttons
    print("✅ buttons_parser")
except ImportError as e:
    print(f"❌ buttons_parser: {e}")

try:
    from oculus_reader_simple import OculusReader
    print("✅ oculus_reader_simple")
except ImportError as e:
    print(f"❌ oculus_reader_simple: {e}")

# Test IK solver dependencies
try:
    import pinocchio
    print("✅ pinocchio")
except ImportError as e:
    print(f"❌ pinocchio: {e}")

try:
    import casadi
    print("✅ casadi")
except ImportError as e:
    print(f"❌ casadi: {e}")

# Test robot controller dependencies
sys.path.append('/home/dora/dora-pipers/questVR_ws/src/Piper_ros/src/piper/scripts')
try:
    from piper_sdk import C_PiperInterface
    print("✅ piper_sdk")
except ImportError as e:
    print(f"❌ piper_sdk: {e}")

print("-" * 40)
print("Import test complete!")
print("\nTo run the system:")
print("  cd /home/dora/dora-pipers/quest_vr_dora_clean")
print("  ./start_all.sh")