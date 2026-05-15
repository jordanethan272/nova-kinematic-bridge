import math

class NovaSafetySentinel:
    """Parallel Observer for EKF Failsafe Redundancy."""
    def __init__(self, state_dim=3):
        self.mu = [0.0] * state_dim
        self.covariance = [[1.0 if i == j else 0.0 for j in range(state_dim)] for i in range(state_dim)]

    def mahalanobis_distance(self, measurement):
        """Calculates distance of measurement from the expected state distribution."""
        diff = [measurement[i] - self.mu[i] for i in range(len(measurement))]
        
        # Simple diagonal inverse for alpha version
        inv_cov = [[1.0/self.covariance[i][i] if i == j else 0.0 for j in range(len(self.mu))] for i in range(len(self.mu))]
        
        # distance = sqrt(diff^T * inv_cov * diff)
        dist_sq = sum(diff[i] * inv_cov[i][i] * diff[i] for i in range(len(diff)))
        return math.sqrt(dist_sq)

    def verify_state(self, measurement, threshold=3.0):
        """Triggers alarm if distance exceeds standard deviations."""
        distance = self.mahalanobis_distance(measurement)
        if distance > threshold:
            return False, distance # FAIL
        return True, distance # PASS

if __name__ == "__main__":
    nss = NovaSafetySentinel(state_dim=3) # [Roll, Pitch, Yaw]
    # Normal state
    print(f"Normal Check: {nss.verify_state([0.1, 0.1, 0.1])}")
    # Compromised Gyro Check (Sudden 10 rad/s spike)
    print(f"Compromised Check: {nss.verify_state([10.0, 0.1, 0.1])}")
