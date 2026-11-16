"""Batched, motor-level Genesis drone environment for future MPPI rollouts."""

import math
from dataclasses import dataclass, fields

import genesis as gs
import torch

from backend.course import Course, CourseGeometry


@dataclass(frozen=True)
class EnvConfig:
    n_envs: int = 1
    physics_dt: float = 0.01
    physics_substeps: int = 2
    control_steps: int = 2
    max_rpm: float = 22000.0
    drone_radius: float = 0.08
    show_viewer: bool = False
    render: bool = False
    validate_actions: bool = True
    hover_after_finish: bool = True
    hover_speed_tolerance: float = 0.25
    hover_angular_velocity_tolerance: float = 0.5

    def __post_init__(self):
        if self.n_envs < 1 or self.control_steps < 1 or self.physics_substeps < 1:
            raise ValueError("n_envs, control_steps, and physics_substeps must be positive")
        if self.physics_dt <= 0 or self.max_rpm <= 0:
            raise ValueError("physics_dt and max_rpm must be positive")
        if not all(
            math.isfinite(v) and v > 0
            for v in (
                self.hover_speed_tolerance,
                self.hover_angular_velocity_tolerance,
            )
        ):
            raise ValueError("Hover tolerances must be finite and positive")


@dataclass
class EnvState:
    """Physical and task state; tensors keep an explicit environment dimension.

    qpos and dofs_velocity are opaque Genesis coordinates. Use observations for
    world-frame velocities. Motor commands are instantaneous, with no motor lag.
    Propeller animation phase and the scene's global step count are not restored;
    neither affects these collision-free drone dynamics or task timing.
    """

    qpos: torch.Tensor
    dofs_velocity: torch.Tensor
    next_gate: torch.Tensor
    elapsed_steps: torch.Tensor
    failed: torch.Tensor
    finished: torch.Tensor
    finish_time: torch.Tensor
    previous_rpm: torch.Tensor

    def select(self, index: int):
        """A single-row state suitable for broadcasting to every rollout."""
        return EnvState(
            **{f.name: getattr(self, f.name)[index : index + 1].clone() for f in fields(self)}
        )


@dataclass
class Transition:
    """One physics substep, with one row per environment.

    active and racing describe the phase BEFORE stepping. hovering describes
    the phase AFTER stepping, so braking starts on the final crossing itself.
    passed/finished/failed are one-time events, not latched episode flags.
    gate_center is the old target, even when this transition passes that gate.
    """

    previous_position: torch.Tensor
    position: torch.Tensor
    gate_center: torch.Tensor
    rpm: torch.Tensor
    previous_rpm: torch.Tensor
    velocity: torch.Tensor
    angular_velocity: torch.Tensor
    yaw_rate: torch.Tensor
    active: torch.Tensor
    racing: torch.Tensor
    hovering: torch.Tensor
    passed: torch.Tensor
    finished: torch.Tensor
    failed: torch.Tensor


def body_yaw_rate(quaternion, angular_velocity):
    """Project world angular velocity onto body Z (wxyz unit quaternion)."""
    w, x, y, z = quaternion.unbind(-1)
    body_z = torch.stack((2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)), -1)
    return (angular_velocity * body_z).sum(-1)


class DroneGateEnv:
    """One Genesis scene with independent copies of the same course.

    Call gs.init once before constructing environments. All public tensors live
    on gs.device and retain shape (N, ...), including N=1. Actions are RPM in
    Genesis's prop0, prop1, prop2, prop3 order. Coordinates are metres, Z up;
    quaternions are wxyz and observed velocities are in world coordinates.
    """

    def __init__(self, config: EnvConfig | None = None, course: Course | None = None):
        config = config or EnvConfig()
        self.config, self.course = config, course or Course.load()
        self.n_envs, self.device = config.n_envs, gs.device
        self.dt = config.physics_dt * config.control_steps
        self.geometry = CourseGeometry(self.course, config.drone_radius, self.device)
        camera_pos, camera_lookat = self.course.overview_camera()
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(
                dt=config.physics_dt, substeps=config.physics_substeps
            ),
            rigid_options=gs.options.RigidOptions(enable_collision=False),
            viewer_options=gs.options.ViewerOptions(
                camera_pos=camera_pos, camera_lookat=camera_lookat, refresh_rate=50
            ),
            vis_options=gs.options.VisOptions(rendered_envs_idx=[0]),
            show_viewer=config.show_viewer,
        )
        self.scene.add_entity(gs.morphs.Plane(collision=False))
        for gate in self.course.gates:
            yaw = math.radians(gate.yaw_degrees)
            for center, size in self.course.bars():
                x, y, z = center
                world_center = (
                    gate.center[0] + math.cos(yaw) * x - math.sin(yaw) * y,
                    gate.center[1] + math.sin(yaw) * x + math.cos(yaw) * y,
                    gate.center[2] + z,
                )
                self.scene.add_entity(
                    gs.morphs.Box(
                        pos=world_center,
                        size=size,
                        euler=(0, 0, gate.yaw_degrees),
                        fixed=True,
                        collision=False,
                    ),
                    surface=gs.surfaces.Default(color=gate.color),
                )
        self.drone = self.scene.add_entity(
            gs.morphs.Drone(
                file="urdf/drones/cf2x.urdf",
                pos=self.course.start,
                default_armature=0.0,
                default_base_ang_damping_scale=0.0,
            )
        )
        self.camera = None
        if config.render:
            self.camera = self.scene.add_camera(
                res=(960, 640),
                pos=camera_pos,
                lookat=camera_lookat,
                fov=50,
                GUI=False,
            )
        # Always build a batched scene, including for one drone. env_spacing is
        # only a display offset; state coordinates remain local to each course.
        self.scene.build(n_envs=self.n_envs, env_spacing=(0.0, 4.0))
        self.mass = float(self.drone.get_mass().flatten()[0])
        self.hover_rpm = math.sqrt(self.mass * 9.81 / (4 * self.drone.KF))
        self.next_gate = torch.zeros(self.n_envs, dtype=torch.long, device=self.device)
        self.elapsed_steps = torch.zeros_like(self.next_gate)
        self.failed = torch.zeros(self.n_envs, dtype=torch.bool, device=self.device)
        self.finished = torch.zeros_like(self.failed)
        self.finish_time = torch.full((self.n_envs,), torch.nan, device=self.device)
        self.previous_rpm = torch.full((self.n_envs, 4), self.hover_rpm, device=self.device)
        self._initial_state = self.get_state()

    @property
    def done(self):
        return self.failed if self.config.hover_after_finish else self.failed | self.finished

    def observe(self):
        position, quaternion = self.drone.get_pos(), self.drone.get_quat()
        velocity, angular_velocity = self.drone.get_vel(), self.drone.get_ang()
        hover_phase = self.finished & ~self.failed & self.config.hover_after_finish
        hover_settled = (
            hover_phase
            & (torch.linalg.vector_norm(velocity, dim=-1) <= self.config.hover_speed_tolerance)
            & (
                torch.linalg.vector_norm(angular_velocity, dim=-1)
                <= self.config.hover_angular_velocity_tolerance
            )
        )
        return {
            "position": position,
            "quaternion": quaternion,
            "velocity": velocity,
            "angular_velocity": angular_velocity,
            "next_gate": self.next_gate.clone(),
            "elapsed_time": self.elapsed_steps * self.config.physics_dt,
            "failed": self.failed.clone(),
            "finished": self.finished.clone(),
            "finish_time": self.finish_time.clone(),
            "previous_rpm": self.previous_rpm.clone(),
            "hover_phase": hover_phase,
            "hover_settled": hover_settled,
            "hover_time": torch.where(
                self.finished, self.elapsed_steps * self.config.physics_dt - self.finish_time, 0.0
            ),
        }

    def get_state(self):
        return EnvState(
            qpos=self.drone.get_qpos().clone(),
            dofs_velocity=self.drone.get_dofs_velocity().clone(),
            **{
                name: getattr(self, name).clone()
                for name in (
                    "next_gate",
                    "elapsed_steps",
                    "failed",
                    "finished",
                    "finish_time",
                    "previous_rpm",
                )
            },
        )

    @torch.inference_mode()
    def set_state(self, state: EnvState, envs_idx=None):
        """Restore N rows, or broadcast one row, to all or selected environments.

        For example: rollouts.set_state(live.get_state().select(0)). A subset
        restore expects one row or len(envs_idx) rows, in that index order.
        """
        ids = (
            torch.arange(self.n_envs, device=self.device)
            if envs_idx is None
            else torch.as_tensor(envs_idx, device=self.device, dtype=torch.long)
        )
        if ids.ndim != 1 or ids.numel() == 0:
            raise ValueError("envs_idx must be a nonempty vector")
        values = {}
        for state_field in fields(state):
            value = getattr(state, state_field.name).to(self.device)
            expected = getattr(self._initial_state, state_field.name).shape[1:]
            if value.shape[1:] != expected or value.shape[0] not in (1, len(ids)):
                raise ValueError(
                    f"Invalid shape for state.{state_field.name}: {tuple(value.shape)}"
                )
            values[state_field.name] = value.expand(len(ids), *expected).contiguous()
        self.drone.set_qpos(values.pop("qpos"), envs_idx=ids, zero_velocity=False)
        self.drone.set_dofs_velocity(values.pop("dofs_velocity"), envs_idx=ids)
        for name, value in values.items():
            getattr(self, name)[ids] = value

    def reset(self, envs_idx=None):
        state = self._initial_state
        if envs_idx is not None:
            ids = torch.as_tensor(envs_idx, device=self.device, dtype=torch.long)
            state = EnvState(**{f.name: getattr(state, f.name)[ids] for f in fields(state)})
        self.set_state(state, envs_idx)
        return self.observe()

    def compute_reward(self, transition: Transition):
        """Override in a task subclass; return named reward tensors of shape (N,).

        Called after EACH physics substep, before previous_rpm is updated.
        The base environment provides physics and events, with zero reward.
        """
        return {}

    @torch.inference_mode()
    def step(self, rpm):
        """Return (observation, reward, info) after one control interval.

        Input shape is (N, 4); a (4,) vector broadcasts to every environment.
        Finite RPMs are clipped to [0, max_rpm]. Task terminal flags latch until
        reset. By default passing the last gate starts an ongoing hover phase;
        only failure is terminal. Set hover_after_finish=False for race-only
        episodes. Terminal rows keep advancing physically but earn zero reward.
        info includes event masks and signed, weighted reward_terms (N,).
        No automatic resets or loops over environments. Disable
        validate_actions to skip the finite-input check and its host sync.
        """
        rpm = torch.as_tensor(rpm, dtype=torch.float32, device=self.device)
        if rpm.shape == (4,):
            rpm = rpm.expand(self.n_envs, 4)
        if rpm.shape != (self.n_envs, 4):
            raise ValueError(f"RPM shape must be ({self.n_envs}, 4) or (4,)")
        if self.config.validate_actions and not bool(torch.isfinite(rpm).all()):
            raise ValueError("RPM commands must be finite")
        rpm = rpm.clamp(0, self.config.max_rpm).contiguous()
        passed_any = torch.zeros_like(self.failed)
        collision_any = torch.zeros_like(self.failed)
        missed_any = torch.zeros_like(self.failed)
        terms = {}
        for _ in range(self.config.control_steps):
            active = ~self.done
            racing = active & ~self.finished
            previous = self.drone.get_pos()
            target = self.geometry.centers[self.next_gate.clamp_max(len(self.course.gates) - 1)]
            self.drone.set_propellers_rpm(rpm)
            self.scene.step()
            position = self.drone.get_pos()
            passed, frame_hit, missed = self.geometry.check(previous, position, self.next_gate)
            ground_hit = position[:, 2] <= self.config.drone_radius
            velocity = self.drone.get_vel()
            angular_velocity = self.drone.get_ang()
            yaw_rate = body_yaw_rate(self.drone.get_quat(), angular_velocity)
            invalid = (
                ~torch.isfinite(position).all(-1)
                | ~torch.isfinite(velocity).all(-1)
                | ~torch.isfinite(angular_velocity).all(-1)
                | ~torch.isfinite(yaw_rate)
            )
            collision = (frame_hit | ground_hit | invalid) & active
            just_failed = collision | (missed & active)
            self.failed |= just_failed
            passed &= active & ~self.failed
            self.next_gate += passed.long()
            self.elapsed_steps += active.long()
            just_finished = (self.next_gate == len(self.course.gates)) & racing & ~self.failed
            self.finished |= just_finished
            self.finish_time = torch.where(
                just_finished, self.elapsed_steps * self.config.physics_dt, self.finish_time
            )
            transition = Transition(
                previous_position=previous,
                position=position,
                gate_center=target,
                rpm=rpm,
                previous_rpm=self.previous_rpm,
                active=active,
                passed=passed,
                finished=just_finished,
                failed=just_failed,
                yaw_rate=yaw_rate,
                racing=racing,
                hovering=self.finished & active & ~self.failed & self.config.hover_after_finish,
                velocity=velocity,
                angular_velocity=angular_velocity,
            )
            for name, value in self.compute_reward(transition).items():
                if name not in terms:
                    terms[name] = torch.zeros_like(value)
                terms[name] += value
            self.previous_rpm = torch.where(active[:, None], rpm, self.previous_rpm)
            passed_any |= passed
            collision_any |= collision
            missed_any |= missed & active
        reward = (
            torch.stack(tuple(terms.values())).sum(0)
            if terms
            else torch.zeros(self.n_envs, device=self.device)
        )
        return (
            self.observe(),
            reward,
            {
                "passed_gate": passed_any,
                "collision": collision_any,
                "missed_gate": missed_any,
                "done": self.done.clone(),
                "reward_terms": terms,
            },
        )
