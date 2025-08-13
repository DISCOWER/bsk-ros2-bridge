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
    r_hat = rN/np.linalg.norm(rN)
    h_vec = np.cross(rN, vN)
    h_hat = h_vec/np.linalg.norm(h_vec)
    theta_hat = np.cross(r_hat, h_hat)
    theta_hat = theta_hat/np.linalg.norm(theta_hat)
    rotMat = np.column_stack((r_hat, theta_hat, h_hat))

    # Mean motion of the chief spacecraft
    omega_mag = np.sqrt(mu / np.linalg.norm(rN)**3)
    omega_vec = omega_mag * h_hat

    # Convert to inertial frame
    r = rN + rotMat @ r_hill
    v = vN + rotMat @ v_hill + np.cross(omega_vec, rotMat @ r_hill)
    return r, v