import math

import numpy as np


class Rotation:
    """Small scipy.spatial.transform.Rotation subset used by robot runtime."""

    __slots__ = ("_matrix", "_stacked", "_quat")

    def __init__(self, matrix, stacked=False, quat=None):
        self._matrix = np.asarray(matrix, dtype=float)
        self._stacked = stacked
        self._quat = quat

    @classmethod
    def from_euler(cls, axis, angle, degrees=False):
        if axis not in ("x", "y", "z"):
            raise NotImplementedError("Only single-axis x/y/z rotations are supported")

        stacked = _is_sequence(angle)
        if stacked:
            if len(angle) != 1:
                raise NotImplementedError("Only a single Euler angle is supported")
            angle = angle[0]

        if degrees:
            angle = math.radians(angle)

        c = math.cos(angle)
        s = math.sin(angle)
        if axis == "x":
            matrix = ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))
        elif axis == "y":
            matrix = ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
        else:
            matrix = ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))

        return cls(matrix, stacked=stacked)

    @classmethod
    def from_quat(cls, quat):
        if len(quat) != 4:
            raise ValueError("Quaternion must contain x, y, z, w")

        x, y, z, w = (float(value) for value in quat)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0:
            raise ValueError("Zero-norm quaternion is not a valid rotation")

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        matrix = (
            (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        )
        return cls(matrix, quat=(x, y, z, w))

    def apply(self, vectors):
        rotated = np.asarray(vectors, dtype=float) @ self._matrix.T
        if self._stacked and rotated.ndim == 1:
            return rotated.reshape(1, 3)
        return rotated

    def as_euler(self, sequence, degrees=False):
        if sequence != "xyz":
            raise NotImplementedError("Only xyz Euler output is supported")

        if self._quat is not None:
            x, y, z, w = self._quat
            ax = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            ay_arg = 2.0 * (w * y - z * x)
            ay = math.asin(max(-1.0, min(1.0, ay_arg)))
            az = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        else:
            matrix = self._matrix
            ay = math.asin(max(-1.0, min(1.0, -matrix[2, 0])))
            cy = math.cos(ay)
            if abs(cy) > 1e-12:
                ax = math.atan2(matrix[2, 1], matrix[2, 2])
                az = math.atan2(matrix[1, 0], matrix[0, 0])
            else:
                ax = 0.0
                az = math.atan2(-matrix[0, 1], matrix[1, 1])

        if degrees:
            return np.array([math.degrees(ax), math.degrees(ay), math.degrees(az)])
        return np.array([ax, ay, az])


def _is_sequence(value):
    return isinstance(value, (list, tuple, np.ndarray))
