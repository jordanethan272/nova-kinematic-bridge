import math

class NovaKinematicBridge:
    def __init__(self, segment_lengths):
        self.l = segment_lengths

    def get_jacobian(self, theta):
        """Calculates 2x3 Jacobian using pure math for zero-dependency sovereignty."""
        t1, t2, t3 = theta
        l1, l2, l3 = self.l
        
        # Sum of angles for compound joints
        t12 = t1 + t2
        t123 = t1 + t2 + t3
        
        s1, s12, s123 = math.sin(t1), math.sin(t12), math.sin(t123)
        c1, c12, c123 = math.cos(t1), math.cos(t12), math.cos(t123)
        
        # Row 1 (X-velocity components)
        j11 = -l1*s1 - l2*s12 - l3*s123
        j12 = -l2*s12 - l3*s123
        j13 = -l3*s123
        
        # Row 2 (Y-velocity components)
        j21 = l1*c1 + l2*c12 + l3*c123
        j22 = l2*c12 + l3*c123
        j23 = l3*c123
        
        return [[j11, j12, j13], [j21, j22, j23]]

if __name__ == "__main__":
    nkb = NovaKinematicBridge([1.0, 1.0, 1.0])
    # Test at zero configuration (fully extended along X-axis)
    res = nkb.get_jacobian([0, 0, 0])
    print(f"J11: {res[0][0]}, J21: {res[1][0]}")
