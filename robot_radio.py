"""Operator profile intent and timestamped three-position switch selection."""
from __future__ import annotations
from protocol_defs import Profile
from robot_config import ProfileSettings


def calibrated_profile(raw: int, config: ProfileSettings) -> Profile | None:
    if isinstance(raw, bool) or not isinstance(raw, int):
        return None
    for name, profile in (("gentle", Profile.GENTLE), ("normal", Profile.NORMAL), ("boost", Profile.BOOST)):
        low, high = getattr(config, f"{name}_range")
        if low <= raw <= high:
            return profile
    return None


class ProfileSelector:
    """Downgrades immediately; higher envelopes require stable fresh RC frames.

    Invalid or transitioning input never preserves Boost. Repeatedly observing
    one frozen frame cannot complete debounce: callers must supply frame time.
    """
    def __init__(self, config: ProfileSettings):
        self.config = config
        self.selected = Profile.GENTLE
        self.valid = False
        self.candidate: Profile | None = None
        self.candidate_at: float | None = None
        self.last_frame_at: float | None = None
        self.reason = "profile input unavailable"

    def reset(self) -> None:
        self.__init__(self.config)

    def observe(self, raw: int, frame_at: float | None) -> Profile:
        candidate = calibrated_profile(raw, self.config)
        if candidate is None or frame_at is None:
            self.selected = Profile.GENTLE
            self.valid = False
            self.candidate = None
            self.candidate_at = None
            self.reason = "invalid profile switch; Gentle fallback"
            return self.selected
        if self.last_frame_at is not None and frame_at <= self.last_frame_at:
            return self.selected
        self.last_frame_at = frame_at
        if candidate != self.candidate:
            self.candidate = candidate
            self.candidate_at = frame_at
            self.selected = min(self.selected, candidate)
            self.valid = False
        if self.candidate_at is not None and frame_at - self.candidate_at >= self.config.debounce_s:
            self.selected = candidate
            self.valid = True
            self.reason = "stable calibrated profile"
        else:
            self.reason = "profile switch settling; lower envelope"
        return self.selected
