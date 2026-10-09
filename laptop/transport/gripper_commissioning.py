"""Bounded nominal +1 mm gripper opening for mapping commissioning."""
from dataclasses import dataclass
import struct
from laptop.transport.position_commands import CanTxFrame


@dataclass(frozen=True)
class OemGripperTestFrame:
    """Captured OEM opening endpoint, bounded five-second send window."""
    approved_monotonic_ns: int
    feedback_monotonic_ns: int
    target_raw: int = 54015

    def validate(self, now_ns):
        if type(self.target_raw) is not int or self.target_raw != 54015:
            raise ValueError('only captured OEM 54.015mm endpoint permitted')
        if any(type(v) is not int or v <= 0 for v in (self.approved_monotonic_ns,self.feedback_monotonic_ns,now_ns)):
            raise ValueError('positive integer clocks required')
        if not 0 <= now_ns-self.approved_monotonic_ns <= 5_000_000_000:
            raise PermissionError('OEM test send window expired')
        if not 0 <= now_ns-self.feedback_monotonic_ns <= 120_000_000:
            raise PermissionError('fresh gripper feedback required')

    @property
    def frame(self):
        return CanTxFrame(0x159,struct.pack('>iHBB',self.target_raw,0,1,0))


@dataclass(frozen=True)
class GripperOpeningStep:
    source_raw: int
    source_monotonic_ns: int
    approved_monotonic_ns: int
    delta_raw: int = 1000
    effort_raw: int = 100
    close_to_zero: bool = False
    open_to_max: bool = False

    def validate(self, now_ns):
        if type(self.open_to_max) is not bool or (self.open_to_max and self.close_to_zero):
            raise ValueError("choose exactly one explicit gripper endpoint scope")
        if type(self.close_to_zero) is not bool:
            raise ValueError("close_to_zero must be boolean")
        if self.open_to_max:
            if self.delta_raw != 0 or self.effort_raw != 100:
                raise ValueError("maximum opening uses explicit 70mm target at 0.1Nm only")
        elif self.close_to_zero:
            if self.delta_raw != 0 or self.effort_raw != 100 or not 0 <= self.source_raw <= 6000:
                raise ValueError("closure requires 0..6mm source and 0.1Nm effort")
        elif (type(self.delta_raw) is not int or type(self.effort_raw) is not int
                or (self.delta_raw, self.effort_raw) not in ((1000, 100), (5000, 300))):
            raise ValueError("only explicit +1 mm/0.1 N.m or +5 mm/0.3 N.m scopes permitted")
        if type(self.source_raw) is not int or not 0 <= self.source_raw <= 70000-self.delta_raw:
            raise ValueError("gripper source must permit a +1 mm target within nominal 0-70 mm")
        if any(type(v) is not int or v <= 0 for v in
               (self.source_monotonic_ns, self.approved_monotonic_ns, now_ns)):
            raise ValueError("gripper clocks must be positive integers")
        if not 0 <= now_ns - self.source_monotonic_ns <= 120_000_000:
            raise PermissionError("gripper feedback is stale or in the future")
        if not 0 <= now_ns - self.approved_monotonic_ns <= 200_000_000:
            raise PermissionError("gripper approval is expired or in the future")

    @property
    def target_raw(self):
        if self.open_to_max:
            return 70000
        return 0 if self.close_to_zero else self.source_raw + self.delta_raw

    @property
    def frame(self):
        # Nominal 0.1 N.m effort, enable=1, no zero-setting flag.
        return CanTxFrame(0x159, struct.pack(">iHBB", self.target_raw, self.effort_raw, 1, 0))
