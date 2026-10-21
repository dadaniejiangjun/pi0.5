"""TRAC-IK adapter using Gamepad_PiPER's solver/seed pattern.

Reference: kehuanjack/Gamepad_PiPER at aa00c80, src/kinematic_trac_ik.py.
No hardware API is used here. The wheel is installed in a project-local prefix.
"""
import math

from .kinematics import PiperKinematics, _matvec, _rotation_vector, _matmul, _transpose


class TracIKPiperKinematics(PiperKinematics):
    use_feedback_solver_limits = True

    def __init__(self, urdf_path, tool_translation_m=(0., 0., 0.), *, solver_type="Speed"):
        super().__init__(urdf_path, tool_translation_m)
        import numpy as np
        from trac_ik import TracIK
        self.np = np
        self.solver = TracIK(
            base_link_name="base_link", tip_link_name="link6",
            urdf_path=str(self.urdf_path), timeout=0.005,
            epsilon=0.00001, solver_type=solver_type)
        if self.solver.dof != 6:
            raise ValueError("TRAC-IK must use exactly the six PiPER arm joints")
        self.previous_solution = None
        self._solver_limits = None
        self.last_solver = "trac_ik"

    def inverse(self, target, seed_rad, *, joint_limits_rad=None, **_kwargs):
        limits = tuple(joint_limits_rad or self.urdf_limits_rad)
        if (len(seed_rad) != 6 or len(limits) != 6
                or any(not math.isfinite(v) for v in seed_rad)
                or any(not math.isfinite(a+b) or a >= b for a,b in limits)):
            raise ValueError("invalid TRAC-IK seed or limits")
        if any(not low <= q <= high for q,(low,high) in zip(seed_rad,limits)):
            raise ValueError("TRAC-IK seed outside effective joint limits")
        if limits != self._solver_limits:
            self.solver.joint_limits = (
                [a for a,b in limits], [b for a,b in limits])
            self._solver_limits = limits
        # Previous solutions are seeds only; distant stored poses never become
        # targets or override the caller's measured-feedback envelope.
        seed = seed_rad
        if self.previous_solution is not None and all(
                low <= old <= high and abs(old-current) <= math.radians(5)
                for old,current,(low,high)
                in zip(self.previous_solution,seed_rad,limits)):
            seed = self.previous_solution
        offset = _matvec(target.rotation, self.tool_translation_m)
        tip = self.np.asarray([p-v for p,v in zip(target.position_m,offset)])
        rotation = self.np.asarray(target.rotation)
        if tip.shape != (3,) or rotation.shape != (3,3) or not (
                self.np.isfinite(tip).all() and self.np.isfinite(rotation).all()):
            raise ValueError("invalid finite TRAC-IK target pose")
        result = self.solver.ik(tip, rotation, seed_jnt_values=self.np.asarray(seed))
        if result is None:
            raise ValueError("TRAC-IK found no solution within its 5 ms budget")
        self.last_solver = "trac_ik"
        q = tuple(float(v) for v in result)
        if len(q) != 6 or any(not math.isfinite(v) or not low <= v <= high
                             for v,(low,high) in zip(q,limits)):
            raise ValueError("TRAC-IK returned invalid or out-of-limit joints")
        actual = self.forward(q)
        error_r = _rotation_vector(_matmul(target.rotation,_transpose(actual.rotation)))
        if (math.dist(actual.position_m,target.position_m) > _kwargs.get('position_tolerance_m',2e-5)
                or math.sqrt(sum(v*v for v in error_r)) > _kwargs.get('orientation_tolerance_rad',2e-4)):
            raise ValueError("TRAC-IK result failed independent FK residual check")
        return q

    def accept_solution(self, q):
        # Only a candidate admitted by the outer feedback envelope may seed
        # the next call. Rejected distant solutions must never pollute it.
        self.previous_solution = tuple(q)
