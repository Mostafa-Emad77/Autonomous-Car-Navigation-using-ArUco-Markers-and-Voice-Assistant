import numpy as np

class EKFSLAM:
    def __init__(self, init_state=np.zeros(3), init_covariance=np.zeros((3, 3))):
        self.mu = init_state.copy()
        self.Sigma = init_covariance.copy()
        self.ids = []

    def add_landmark(self, landmark_position_camera: tuple, id: str):
        self.ids.append(id)

        # Expand state vector to accommodate the new landmark
        new_mu = np.zeros(self.mu.shape[0] + 2)
        new_mu[:-2] = self.mu
        new_mu[-2:] = landmark_position_camera
        self.mu = new_mu

        # Expand covariance matrix accordingly
        new_Sigma = np.zeros((self.Sigma.shape[0] + 2, self.Sigma.shape[0] + 2))
        new_Sigma[:-2, :-2] = self.Sigma
        initial_landmark_covariance = 10**2  # Initial covariance for new landmark
        new_Sigma[-2:, -2:] = np.array([[initial_landmark_covariance, 0], [0, initial_landmark_covariance]])
        self.Sigma = new_Sigma

    def correction(self, landmark_positions_measured: list, landmark_ids: list):
        for i, landmark_id in enumerate(landmark_ids):
            # Find index of the landmark in the state vector
            index = self.ids.index(landmark_id)

            # Extract measured landmark position
            landmark_position_measured = landmark_positions_measured[i]

            # Compute Kalman gain
            H = self.jacobian_h(index)
            Q = np.eye(2) * 1.0  # Measurement covariance (adjust as needed)
            K = self.Sigma @ H.T @ np.linalg.inv(H @ self.Sigma @ H.T + Q)

            # Compute measurement residual
            z_pred = self.mu[3 + 2 * index : 3 + 2 * index + 2]  # Predicted landmark position
            z = landmark_position_measured
            delta_z = z - z_pred

            # Update state estimate
            self.mu = self.mu + K @ delta_z

            # Update covariance matrix
            self.Sigma = (np.eye(len(self.mu)) - K @ H) @ self.Sigma

    def jacobian_h(self, i):
        # Extract robot's pose
        x = self.mu[0]
        y = self.mu[1]
        theta = self.mu[2]

        # Extract landmark's position
        xm = self.mu[3 + 2 * i]
        ym = self.mu[3 + 2 * i + 1]

        # Compute some intermediate values
        dx = xm - x
        dy = ym - y
        q = dx**2 + dy**2
        sqrt_q = np.sqrt(q)

        # Compute Jacobian matrix H
        H = np.zeros((2, self.mu.shape[0]))
        H[0, 0] = -sqrt_q * dx
        H[0, 1] = -sqrt_q * dy
        H[0, 2] = 0
        H[0, 3 + 2 * i] = sqrt_q * dx
        H[0, 4 + 2 * i] = sqrt_q * dy

        H[1, 0] = dy
        H[1, 1] = -dx
        H[1, 2] = -q
        H[1, 3 + 2 * i] = -dy
        H[1, 4 + 2 * i] = dx

        H /= q

        return H

    def get_robot_pose(self):
        if np.isnan(np.sum(self.Sigma)):
            print("Covariance matrix contains NaN values. Handling them...")
            self.Sigma = np.nan_to_num(self.Sigma)

        robot_x, robot_y, robot_theta = self.mu[:3].copy()
        robot_stdev = self.get_error_ellipse(self.Sigma[:2, :2])
        return robot_x, robot_y, robot_theta, robot_stdev

    def get_landmark_poses(self):
        landmark_estimated_positions = self.mu[3:]
        landmark_estimated_positions = landmark_estimated_positions.reshape(landmark_estimated_positions.shape[0]//2,2)

        landmark_estimated_stdevs = []
        for i in range(3, self.Sigma.shape[0]-1, 2):
            landmark_estimated_stdevs.append(self.get_error_ellipse(self.Sigma[i:i+2, i:i+2]))
        return landmark_estimated_positions, landmark_estimated_stdevs

    def get_error_ellipse(self, covariance):
        covariance = (covariance + covariance.T) / 2.0
        eigenvalues, eigenvectors = np.linalg.eig(covariance)

        sort_indices = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[sort_indices]
        eigenvectors = eigenvectors[:, sort_indices]

        lambda1,lambda2 = eigenvalues
        v1, v2 = eigenvectors[:, 0], eigenvectors[:, 1]

        angle = np.arctan2(float(v1[1]), float(v1[0]))
        size1 = np.sqrt(np.abs(lambda1))
        size2 = np.sqrt(np.abs(lambda2))
        return size1, size2, angle

    def get_landmark_ids(self):
        return np.array(self.ids)
