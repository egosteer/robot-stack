import numpy as np

def rot_matrix_from_6drot(rot):
    """
    Convert a 6D rotation representation to a 3x3 rotation matrix (NumPy).
    Input can be (6,) or (..., 6).
    """
    rot = np.asanyarray(rot)
    original_shape = rot.shape

    # Reshape to [-1, 6] for uniform batch handling
    rot_reshaped = rot.reshape(-1, 6)
    a1 = rot_reshaped[:, :3]
    a2 = rot_reshaped[:, 3:]

    eps = 1e-10

    # Gram-Schmidt orthogonalization
    b1 = a1 / (np.linalg.norm(a1, axis=-1, keepdims=True) + eps)

    dot_product = np.sum(b1 * a2, axis=-1, keepdims=True)
    b2 = a2 - dot_product * b1
    b2 = b2 / (np.linalg.norm(b2, axis=-1, keepdims=True) + eps)

    b3 = np.cross(b1, b2, axis=-1)

    # Stack b1, b2, b3 as columns: [N, 3, 3]
    rot_matrix = np.stack([b1, b2, b3], axis=-1)

    # Restore original dimensions
    if len(original_shape) > 1:
        return rot_matrix.reshape(*original_shape[:-1], 3, 3)
    else:
        return rot_matrix.reshape(3, 3)

def rot_matrix_to_6drot(rot_matrix):
    """
    Convert a 3x3 rotation matrix to a 6D rotation representation.
    Input can be (3, 3) or (..., 3, 3).
    """
    rot_matrix = np.asanyarray(rot_matrix)
    # Extract and concatenate the first two columns
    a1 = rot_matrix[..., :3, 0]
    a2 = rot_matrix[..., :3, 1]
    return np.concatenate([a1, a2], axis=-1)

def homo_matrix_from_trans_6drot(trans, rot_6d):
    """
    Build a 4x4 homogeneous transform from a translation vector and a 6D rotation.
    Input shapes: (3,) & (6,)  -> returns (4, 4)
    Input shapes: (..., 3) & (..., 6) -> returns (..., 4, 4)
    """
    trans = np.asanyarray(trans)
    rot_6d = np.asanyarray(rot_6d)

    assert trans.shape[:-1] == rot_6d.shape[:-1], "trans and rot_6d must have same prefix shape"

    rot_matrix = rot_matrix_from_6drot(rot_6d)

    prefix_shape = trans.shape[:-1]
    homo_matrix = np.zeros(prefix_shape + (4, 4), dtype=trans.dtype)

    homo_matrix[..., :3, :3] = rot_matrix
    homo_matrix[..., :3, 3] = trans
    homo_matrix[..., 3, 3] = 1.0

    return homo_matrix

def homo_matrix_to_trans_6drot(homo_matrix):
    """
    Extract a translation vector and 6D rotation from a 4x4 homogeneous transform.
    """
    homo_matrix = np.asanyarray(homo_matrix)

    trans = homo_matrix[..., :3, 3]
    rot_matrix = homo_matrix[..., :3, :3]
    rot_6d = rot_matrix_to_6drot(rot_matrix)
    
    return trans, rot_6d