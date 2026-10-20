"""Nonblocking F6 session: align simulation, reanchor the goal, enter TELE_OP."""

import math
import time

from .joint_mapping import map_sim_to_robot
from .teleop_gui import ROBOT_JOINTS, SIM_JOINTS


class RobotConnection:
    def __init__(self, node, hand_client, mapping_provider=None):
        self.node, self.hand = node, hand_client
        self.mapping_provider = mapping_provider or (lambda: node.mapping)
        self.phase = "off"
        self.message = "Robot teleoperation OFF. F6 connects after simulation alignment."
        self.error = False
        self._deadline = 0.0
        self._future = None
        self._mode = None
        self._runtime_id = None
        self._mapping = None
        self._aligned_at = 0.0
        self._sync_sequence = None
        self._stop_reason = ""
        self._stop_error = False
        self.follow_pending = False
        self._follow_deadline = 0.0
        node.declare_parameter("robot_alignment_tolerance_deg", 1.0)
        self.tolerance = math.radians(float(node.get_parameter("robot_alignment_tolerance_deg").value))
        if not math.isfinite(self.tolerance) or self.tolerance <= 0:
            raise ValueError("robot_alignment_tolerance_deg must be positive and finite")

    @staticmethod
    def _fresh_feedback(state):
        now = time.monotonic()
        return (state["robot_q"] is not None and now - state["robot_time"] < 0.3
                and now - state["robot_status_time"] < 1.25)

    @staticmethod
    def _fresh_simulation(state, hand):
        return (state["sim_q"] is not None and time.monotonic() - state["sim_time"] < 0.3
                and hand["simulation_fresh"] and hand["simulation_current"] is not None)

    def snapshot(self):
        state, hand = self.node.snapshot(), self.hand.snapshot()
        raw_active = state["armed"] or state["robot_status"].startswith("arming")
        active = self.phase != "off" or raw_active
        preparing = self.phase not in ("off", "connected", "stop_failed") or self.follow_pending
        ready = (self._fresh_feedback(state) and self.node.arm_client.service_is_ready()
                 and self._fresh_simulation(state, hand) and not hand["pending"]
                 and not hand["run_held"] and not hand["simulation_running"])
        if hand["input_mode"] == "topic":
            ready = ready and hand["status_fresh"] and hand["status"].get("can_sync", False)
        detail = self.message
        if not active and not self.error:
            if not self.node.arm_client.service_is_ready():
                detail = "Robot driver unavailable. Start the DCP3 driver to enable F6."
            elif not self._fresh_feedback(state):
                detail = "Waiting for fresh robot feedback."
            elif not self._fresh_simulation(state, hand):
                detail = "Waiting for fresh simulation joint states."
            elif hand["run_held"] or hand["simulation_running"]:
                detail = "Release F7 before connecting the robot."
            elif hand["input_mode"] == "topic" and not hand["status"].get("can_sync"):
                detail = "Topic Pose input must be ready for F6 re-mapping, or select Keyboard."
        if self.phase == "off" and raw_active:
            detail = "Robot TELE_OP active through manual controls. F6 disconnects."
        return {"phase": self.phase, "active": active, "preparing": preparing,
                "connected": self.phase == "connected", "message": detail, "error": self.error,
                "can_toggle": bool(active or ready),
                "allow_run": self.phase in ("off", "connected") and not self.follow_pending,
                "lock_settings": active}

    def begin_follow(self):
        if self.phase != "connected" or not self.hand.snapshot()["can_run_simulation"]:
            return False
        self.node.set_managed_follow(False)
        try:
            self.node.sync_robot_to_sim()
        except ValueError as exc:
            self._fail(str(exc))
            return False
        self.follow_pending = True
        self._follow_deadline = time.monotonic() + 2.0
        self.message = "Matching simulation to measured joints before F7 motion."
        return True

    def cancel_follow(self):
        self.follow_pending = False
        if self.phase == "connected":
            self.node.set_managed_follow(False)

    def connect(self):
        if self.snapshot()["active"] or not self.snapshot()["can_toggle"]:
            return False
        try:
            self.node.set_mapping(self.mapping_provider())
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.message, self.error = f"Joint mapping error: {exc}", True
            return False
        self._mapping = dict(self.node.mapping)
        self._mode = self.hand.snapshot()["input_mode"]
        self._runtime_id = self.hand.snapshot()["simulation"].get("runtime_id")
        self.node.set_stream(False)
        self.node.set_deadman(False)
        if not self.hand.request_run(False):
            self.message, self.error = "Cannot pause the simulation before connecting.", True
            return False
        self.phase, self.error = "pausing", False
        self.message = "Preparing robot connection: pausing simulation."
        self._deadline = time.monotonic() + 3.0
        return True

    def disconnect(self, reason="Robot teleoperation OFF.", *, error=False):
        self.cancel_follow()
        hand = self.hand.snapshot()
        if hand["run_held"] or hand["can_stop_simulation"]:
            self.hand.request_run(False)
        self.node.set_deadman(False)
        self.node.set_stream(False)
        self._stop_reason, self._stop_error = reason, error
        self.phase = "stopping"
        self.message = "Stopping robot teleoperation..."
        self._future = self.node.arm(False)
        self._deadline = time.monotonic() + 4.0
        if self._future is None:
            self.phase, self.error = "stop_failed", True
            self.message = "Robot stop service unavailable; output disabled. F6 retries stop."

    def _fail(self, message):
        self.disconnect(message, error=True)

    def _aligned(self, state):
        if state["sim_q"] is None or time.monotonic() - state["sim_time"] > 0.3:
            return False
        mapped = map_sim_to_robot(state["sim_q"], SIM_JOINTS, ROBOT_JOINTS, self._mapping)
        return max(abs(a - b) for a, b in zip(mapped, state["robot_q"])) <= self.tolerance

    def tick(self):
        """Called only from Qt; ROS callbacks keep running during every phase."""
        if self.phase in ("off", "stop_failed"):
            return
        now = time.monotonic()
        state, hand = self.node.snapshot(), self.hand.snapshot()
        if self.phase == "stopping":
            if self._future is not None and self._future.done():
                try:
                    response = self._future.result()
                    if not response.success:
                        raise RuntimeError(response.message)
                except Exception as exc:
                    self.phase, self.error = "stop_failed", True
                    self.message = f"Robot stop failed: {exc}. F6 retries stop."
                else:
                    self.phase, self.error = "off", self._stop_error
                    self.message = self._stop_reason
                self._future = None
            elif now > self._deadline:
                self.phase, self.error = "stop_failed", True
                self.message = "Robot stop confirmation timed out. F6 retries stop."
            return
        if not self._fresh_feedback(state) or not self._fresh_simulation(state, hand):
            self._fail("Connection stopped: robot or simulation feedback was lost.")
            return
        if hand["input_mode"] != self._mode or self.node.mapping != self._mapping:
            self._fail("Input mode or joint mapping changed. Connect again with F6.")
            return
        if hand["simulation"].get("runtime_id") != self._runtime_id:
            self._fail("Simulation MPC restarted. Connect again with F6.")
            return
        if self.phase != "connected" and (hand["run_held"] or hand["simulation_running"]):
            self._fail("Connection canceled: F7 was pressed during preparation.")
            return
        if self.phase != "connected" and now > self._deadline:
            self._fail(f"Robot connection timed out during {self.phase}.")
            return
        if self.phase == "pausing":
            if hand["pending"] or hand["simulation"].get("execution_enabled"):
                return
            try:
                self.node.sync_robot_to_sim()
            except ValueError as exc:
                self._fail(str(exc))
                return
            self.phase = "aligning"
            self.message = "Aligning simulation joints to measured robot joints."
            self._deadline = now + 3.0
        elif self.phase == "aligning":
            if state["sync_pending"]:
                return
            if not state["sync_success"]:
                self._fail(f"Simulation alignment rejected: {state['sync_result']}")
                return
            if self._aligned(state):
                self._aligned_at = now
                self.phase = "target"
                self.message = "Re-anchoring hand_target to the aligned hand pose."
                self._deadline = now + 4.0
        elif self.phase == "target":
            current = hand["simulation_current"]
            if current is None or current[2] <= self._aligned_at:
                return
            if self._mode == "keyboard":
                if self.hand.reset_keyboard_target():
                    self.phase = "target_ready"
            else:
                mapped_current = hand["current"]
                if mapped_current is None or mapped_current[2] <= self._aligned_at or not hand["can_sync"]:
                    return
                self._sync_sequence = (hand["status"].get("session_id"), hand["status"].get("sync_sequence"))
                if self.hand.request_sync():
                    self.phase = "mapping"
        elif self.phase == "mapping":
            if hand["pending"]:
                return
            if hand["result_kind"] == "error":
                self._fail(f"Pose mapping failed: {hand['result']}")
                return
            key = (hand["status"].get("session_id"), hand["status"].get("sync_sequence"))
            if key != self._sync_sequence and hand["status"].get("calibrated"):
                self.phase = "target_ready"
        elif self.phase == "target_ready":
            if not hand["can_run_simulation"] or not self._aligned(state):
                return
            self.node.set_stream(True)
            self.node.set_managed_follow(False)
            self.node.set_gui_deadman(True)
            self.phase = "priming"
            self.message = "Preparing measured-position hold and Deadman before TELE_OP."
            self._prime_until = now + 0.2
            self._deadline = now + 4.0
        elif self.phase in ("priming", "arming", "connected"):
            if not state["stream_enabled"] or not state["deadman_held"]:
                self._fail("Robot output or Deadman was released. Connect again with F6.")
                return
            self.node.set_gui_deadman(True)
            if self.phase == "priming":
                if not self._aligned(state):
                    self._fail("Robot moved before TELE_OP; align again with F6.")
                elif now >= self._prime_until:
                    self._future = self.node.arm(True)
                    if self._future is None:
                        self._fail("Robot arm service is unavailable.")
                    else:
                        self.phase = "arming"
                        self.message = "Waiting for robot TELE_OP confirmation."
                        self._deadline = now + 4.0
            elif self.phase == "arming":
                if self._future.done():
                    try:
                        response = self._future.result()
                        if not response.success:
                            raise RuntimeError(response.message)
                    except Exception as exc:
                        self._fail(f"Robot TELE_OP rejected: {exc}")
                        return
                    if state["armed"] and state["robot_status"].startswith("armed"):
                        self.phase = "connected"
                        self.message = "Robot TELE_OP ON. Hold F7 to follow hand_target; F6 disconnects."
            else:
                if not state["armed"]:
                    self._fail("Robot left TELE_OP. Connect again with F6.")
                    return
                if self.follow_pending:
                    if now > self._follow_deadline:
                        self._fail("F7 joint alignment timed out.")
                    elif not state["sync_pending"]:
                        if not state["sync_success"]:
                            self._fail(f"F7 alignment rejected: {state['sync_result']}")
                        elif self._aligned(state) and self.hand.request_run(True):
                            self.follow_pending = False
                            self.node.set_managed_follow(True)
                            self.message = "Robot TELE_OP ON. Release F7 to hold measured joints."
                else:
                    self.node.set_managed_follow(hand["run_held"])
                    self.message = ("Robot TELE_OP ON. Release F7 to hold measured joints."
                                    if hand["run_held"] else
                                    "Robot TELE_OP ON. Hold F7 to follow hand_target; F6 disconnects.")

    def focus_lost(self):
        if self.snapshot()["active"] and self.phase not in ("stopping", "stop_failed"):
            self.disconnect("Robot disconnected because the GUI lost focus.")

    def close(self):
        if self.snapshot()["active"] and self.phase != "stopping":
            self.disconnect("Robot disconnected because the GUI closed.")
