import math
from nkb_core import NovaKinematicBridge

class NKBIKSolver:
    def __init__(self, bridge, damping=0.01):
        self.bridge = bridge
        self.damping = damping

    def solve(self, current_theta, target_pos, iterations=100, step_size=0.1):
        """Iterative IK using Damped Least Squares."""
        theta = list(current_theta)
        
        for _ in range(iterations):
            # 1. Forward Kinematics (Calculate current end-effector position)
            # For 3-link planar: x = l1*c1 + l2*c12 + l3*c123, y = l1*s1 + l2*s2 + l3*s3
            l1, l2, l3 = self.bridge.l
            t1, t2, t3 = theta
            t12 = t1 + t2
            t123 = t1 + t2 + t3
            
            curr_x = l1*math.cos(t1) + l2*math.cos(t12) + l3*math.cos(t123)
            curr_y = l1*math.sin(t1) + l2*math.sin(t12) + l3*math.sin(t123)
            
            error_x = target_pos[0] - curr_x
            error_y = target_pos[1] - curr_y
            
            if math.sqrt(error_x**2 + error_y**2) < 0.001:
                break
            
            # 2. Get Jacobian
            J = self.bridge.get_jacobian(theta)
            
            # 3. Damped Least Squares (Simplified for 2x3)
            # Δθ = J^T * inv(J*J^T + λ^2*I) * error
            # J*J^T is a 2x2 matrix
            JJT = [
                [sum(J[0][i]*J[0][i] for i in range(3)) + self.damping**2, sum(J[0][i]*J[1][i] for i in range(3))],
                [sum(J[1][i]*J[0][i] for i in range(3)), sum(J[1][i]*J[1][i] for i in range(3)) + self.damping**2]
            ]
            
            # 2x2 Determinant
            det = JJT[0][0]*JJT[1][1] - JJT[0][1]*JJT[1][0]
            inv_JJT = [
                [JJT[1][1]/det, -JJT[0][1]/det],
                [-JJT[1][0]/det, JJT[0][0]/det]
            ]
            
            # Compute J^T * inv_JJT * error
            # Step 1: inv_JJT * error (result is 2x1)
            temp_x = inv_JJT[0][0]*error_x + inv_JJT[0][1]*error_y
            temp_y = inv_JJT[1][0]*error_x + inv_JJT[1][1]*error_y
            
            # Step 2: J^T * temp (result is 3x1)
            for i in range(3):
                delta_theta = J[0][i]*temp_x + J[1][i]*temp_y
                theta[i] += step_size * delta_theta
                
        return theta

if __name__ == "__main__":
    bridge = NovaKinematicBridge([1.0, 1.0, 1.0])
    solver = NKBIKSolver(bridge)
    target = [1.5, 0.5]
    initial_guess = [0.1, 0.1, 0.1]
    result = solver.solve(initial_guess, target)
    print(f"Target: {target}")
    print(f"Solved Angles: {result}")
