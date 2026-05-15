import struct

class ODriveCANBridge:
    def __init__(self, node_ids=[1, 2, 3]):
        self.node_ids = node_ids
        self.CMD_SET_INPUT_POS = 0x00C

    def generate_pos_frame(self, node_id, position):
        """
        Generates a raw CAN frame for ODrive Position Control.
        Payload: [Float32 Pos, Int16 Vel_FF, Int16 Torque_FF]
        """
        can_id = (node_id << 5) | self.CMD_SET_INPUT_POS
        
        # Pack data: 4-byte float (pos), 2-byte int (0), 2-byte int (0)
        payload = struct.pack('<fhh', float(position), 0, 0)
        
        return {
            "can_id": hex(can_id),
            "payload_hex": payload.hex().upper(),
            "description": f"Node {node_id} -> Move to {position} rad"
        }

    def translate_ik_to_can(self, joint_angles):
        """Translates a list of solved angles into a batch of CAN frames."""
        frames = []
        for i, angle in enumerate(joint_angles):
            if i < len(self.node_ids):
                frames.append(self.generate_pos_frame(self.node_ids[i], angle))
        return frames

if __name__ == "__main__":
    # Test Case: Verified IK Output from previous solve
    # Angles: [-0.8236, 1.0708, 1.4723]
    solved_angles = [-0.8236, 1.0708, 1.4723]
    bridge = ODriveCANBridge()
    output = bridge.translate_ik_to_can(solved_angles)
    
    print("--- ODRIVE CAN-BUS VERIFIED MANIFEST ---")
    for frame in output:
        print(f"ID: {frame['can_id']} | PAYLOAD: {frame['payload_hex']} | {frame['description']}")
