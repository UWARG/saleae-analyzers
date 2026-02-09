import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / "lib"))
import dronecan

binary_payload = "11011001110011100000010100000000000000001011110000000110100111100000000000000000101111000000011100000000000000001011110001111110"
print(len(binary_payload))

print("before packing:", binary_payload)
# --- THE FIX STARTS HERE ---
# _unpack cannot read bytes. We must convert bytes to a list of bits (integers).
# This creates a list of 56 items (bits) from your 7 bytes.
msg = dronecan.uavcan.equipment.actuator.ArrayCommand()
#msg = dronecan.uavcan.protocol.NodeStatus()
print("drone packing:", msg)

msg._unpack(binary_payload) 

print("after unpacking", (dronecan.to_yaml(msg)))