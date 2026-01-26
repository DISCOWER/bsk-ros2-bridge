import numpy as np

def get_initial_conditions_from_hill(mu, rN, vN, r_hill, v_hill):
    """    Convert Hill frame position and velocity to inertial frame.
    Args:
        mu (float): Gravitational parameter of the central body.
        rN (np.ndarray): Position vector in the inertial frame.
        vN (np.ndarray): Velocity vector in the inertial frame.
        r_hill (np.ndarray): Position vector in the Hill frame.
        v_hill (np.ndarray): Velocity vector in the Hill frame.
    Returns:
        r (np.ndarray): Position vector in the inertial frame.
        v (np.ndarray): Velocity vector in the inertial frame.
    """
    # Ensure input vectors are numpy arrays
    rN = np.array(rN)
    vN = np.array(vN)
    r_hill = np.array(r_hill)
    v_hill = np.array(v_hill)
    
    # Rotation matrix from Hill frame to inertial frame
    r_norm = np.linalg.norm(rN)
    r_hat = rN / r_norm
    h_vec = np.cross(rN, vN)
    h_norm = np.linalg.norm(h_vec)
    if h_norm == 0.0:
        raise ValueError("Invalid chief state: rN and vN are colinear; Hill frame undefined")
    h_hat = h_vec / h_norm
    theta_hat = np.cross(h_hat, r_hat)
    theta_hat = theta_hat / np.linalg.norm(theta_hat)
    rotMat = np.column_stack((r_hat, theta_hat, h_hat))

    # Angular velocity of Hill frame
    omega_vec = (h_norm / (r_norm ** 2)) * h_hat

    # Convert to inertial frame
    r = rN + rotMat @ r_hill
    v = vN + rotMat @ v_hill + np.cross(omega_vec, rotMat @ r_hill)
    return r, v