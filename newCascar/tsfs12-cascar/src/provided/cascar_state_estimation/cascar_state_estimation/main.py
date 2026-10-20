"""Compatibility entry point; the ROS implementation is in estimator_node."""

from cascar_state_estimation.estimator_node import EKFStateEstimator, main

__all__ = ["EKFStateEstimator", "main"]

if __name__ == "__main__":
    main()
