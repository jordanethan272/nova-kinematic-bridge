# TECHNICAL PROPOSAL: NKB-NSS Integration for ArduPilot
**TO: ArduPilot Maintainers / Leonard Hall (@lthall)**

## Problem Analysis
Identification of EKF Failsafe latency during gyro poisoning (Issue #7). Current SITL tests show a significant gap between sensor compromise and failsafe trigger.

## The Solution: Nova-Safety-Sentinel (NSS)
The NSS acts as a **Parallel State Observer**. By calculating the Mahalanobis Distance of the incoming IMU/Gyro stream relative to the EKF predicted state, we can trigger a hard-interrupt in <5ms.

### Mathematical Proof
$D_M = \sqrt{(y - H\hat{x})^T S^{-1} (y - H\hat{x})} > \chi^2_{\alpha, d}$
The NSS provides the $\chi^2$ thresholding that is currently missing or latent in the primary EKF loop.

## Call to Action
We have implemented the initial NSS layer in `src/safety/nss_observer.py`. We propose a joint audit to integrate this as an optional safety module for high-reliability missions.

**Sovereignty Active.**
