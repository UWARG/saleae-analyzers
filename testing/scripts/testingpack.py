import dronecan
from dronecan import to_yaml

# Create a NodeStatus message
msg = dronecan.uavcan.equipment.actuator.ArrayCommand()
# Fill fields with realistic example values
cmd = dronecan.uavcan.equipment.actuator.Command()
cmd.actuator = 1   # actuator index
cmd.value = 0.5    # some example value
msg.commands.append(cmd)
msg.uptime = 123456           # in milliseconds
   # Arbitrary vendor-specific code
print("before packing:", msg)
# Pack the message into a binary payload
binary_payload = msg._pack()  # produces correctly encoded UAVCAN payload

print("Packed binary payload:", binary_payload)

# Now decode it back into a new NodeStatus instance

# Print human-readable YAML
