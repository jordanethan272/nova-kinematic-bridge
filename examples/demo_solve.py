from src.nkb_core import NovaKinematicBridge
from src.nkb_ik_solver import NKBIKSolver
from src.nkb_can_bridge import ODriveCANBridge

# 1. Define Robot (3 links, 1.0m each)
bridge_core = NovaKinematicBridge([1.0, 1.0, 1.0])
solver = NKBIKSolver(bridge_core)
can_output = ODriveCANBridge()

# 2. Target Workspace Coordinate
target = [1.2, 0.8]

# 3. Solve & Bridge
angles = solver.solve([0.1, 0.1, 0.1], target)
frames = can_output.translate_ik_to_can(angles)

print(f"Verified Solution for Target {target}:")
for f in frames:
    print(f"CAN ID: {f['can_id']} | DATA: {f['payload_hex']}")
