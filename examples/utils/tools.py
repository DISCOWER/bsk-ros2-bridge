from Basilisk.utilities import RigidBodyKinematics
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

def get_hill_frame_attitude(rN, vN):
    """
    Calculate the Hill frame attitude (as MRP) from position and velocity vectors.
    Hill frame: x=radial out, z=out of orbital plane, y=completes right-handed frame
    
    Args:
        rN (np.ndarray): Position vector in the inertial frame [m]
        vN (np.ndarray): Velocity vector in the inertial frame [m/s]
    
    Returns:
        np.ndarray: MRP (Modified Rodriguez Parameters) for Hill frame attitude
    """
    rN = np.array(rN)
    vN = np.array(vN)
    
    # Calculate Hill frame basis vectors
    r_norm = np.linalg.norm(rN)
    r_hat = rN / r_norm  # x-axis: radial out
    
    h_vec = np.cross(rN, vN)
    h_norm = np.linalg.norm(h_vec)
    if h_norm == 0.0:
        raise ValueError("Invalid state: rN and vN are colinear; Hill frame undefined")
    h_hat = h_vec / h_norm  # z-axis: out of orbital plane
    
    theta_hat = np.cross(h_hat, r_hat)  # y-axis: completes right-handed frame
    theta_hat = theta_hat / np.linalg.norm(theta_hat)
    
    # DCM from inertial to Hill frame (rows are Hill frame axes expressed in inertial frame)
    dcm_HN = np.array([r_hat, theta_hat, h_hat])
    
    # Convert DCM to MRP
    mrp = RigidBodyKinematics.C2MRP(dcm_HN)
    
    return mrp