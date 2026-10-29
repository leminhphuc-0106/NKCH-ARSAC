from __future__ import annotations

from pathlib import Path

import gymnasium as gym
import numpy as np

from stable_baselines3 import SAC
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor

from robot_2r_env import Robot2REnv


class AdaptivePID2REnv(gym.Env):
    """
    Môi trường SAC điều chỉnh hệ số PID cho robot 2R.
    SAC:
        Chọn Kp, Ki, Kd.
    PID:
        Tính torque điều khiển.
    Robot2REnv:
        Nhận torque PID và mô phỏng động lực học robot.
    SAC KHÔNG trực tiếp tạo torque.
    """
    metadata = {"render_modes": []}

    def __init__(self):
        super().__init__()

        # 1. Khởi tạo môi trường robot
        self.robot_env = Robot2REnv(
            dt=0.01,
            episode_seconds=5.0,
            target_q=(np.pi / 4.0, np.pi / 4.0),
            torque_limit=5.0,
            randomize_parameters=True,
            measurement_noise_std=0.001,
        )

        self.dt = float(self.robot_env.dt)
        self.torque_limit = float(self.robot_env.torque_limit)
        # 2. Cấu hình thời gian cập nhật hệ số PID
        self.gain_update_period = 0.05
        self.gain_update_steps = max(
            1,
            int(round(self.gain_update_period / self.dt)),
        )
        self.step_counter = 0
        self.total_time_step = 0
        # =========================================================
        # 3. Giới hạn hệ số PID
        # Mỗi khớp có:
        #     Kp
        #     Ki
        #     Kd
        # Tổng cộng:
        #     6 giá trị
        # =========================================================
        self.kp_min = np.array(
            [0.0, 0.0],
            dtype=np.float32,
        )

        self.kp_max = np.array(
            [100.0, 100.0],
            dtype=np.float32,
        )

        self.ki_min = np.array(
            [0.0, 0.0],
            dtype=np.float32,
        )

        self.ki_max = np.array(
            [100.0, 100.0],
            dtype=np.float32,
        )

        self.kd_min = np.array(
            [0.0, 0.0],
            dtype=np.float32,
        )

        self.kd_max = np.array(
            [100.0, 100.0],
            dtype=np.float32,
        )

        # 4. Giá trị hệ số PID hiện tại
        self.kp = (self.kp_min + self.kp_max) / 2.0
        self.ki = (self.ki_min + self.ki_max) / 2.0
        self.kd = (self.kd_min + self.kd_max) / 2.0

        # 5. Trạng thái PID
        self.integral_error = np.zeros(
            2,
            dtype=np.float32,
        )

        self.integral_limit = 2.0

        # 6. Xác định observation space
        base_low = np.asarray(
            self.robot_env.observation_space.low,
            dtype=np.float32,
        )

        base_high = np.asarray(
            self.robot_env.observation_space.high,
            dtype=np.float32,
        )

        self.base_observation_size = int(base_low.size)

        # Bổ sung:
        #     error_joint_1
        #     error_joint_2
        #     integral_error_joint_1
        #     integral_error_joint_2
        #
        # Tổng cộng thêm 4 giá trị.
        additional_low = np.full(
            4,
            -np.inf,
            dtype=np.float32,
        )

        additional_high = np.full(
            4,
            np.inf,
            dtype=np.float32,
        )

        observation_low = np.concatenate(
            [
                base_low.reshape(-1),
                additional_low,
            ]
        )

        observation_high = np.concatenate(
            [
                base_high.reshape(-1),
                additional_high,
            ]
        )

        self.observation_space = gym.spaces.Box(
            low=observation_low,
            high=observation_high,
            dtype=np.float32,
        )

        # =========================================================
        # 7. Xác định action space
        #
        # SAC tạo ra 6 giá trị trong khoảng [-1, 1].
        # Action:
        #     [Kp1, Ki1, Kd1, Kp2, Ki2, Kd2]
        # Các giá trị này sẽ được chuyển thành hệ số PID
        # thực tế theo giới hạn đã khai báo ở trên.
        # =========================================================
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(6,),
            dtype=np.float32,
        )

    # Chuyển action của SAC thành hệ số PID thực tế
    def action_to_gains(
        self,
        action: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Chuyển action [-1, 1] thành Kp, Ki, Kd.
        Công thức:
            gain = min_gain
                 + (action + 1) / 2
                 * (max_gain - min_gain)
        """

        action = np.asarray(
            action,
            dtype=np.float32,
        )

        action = np.clip(
            action,
            -1.0,
            1.0,
        )

        normalized_action = (action + 1.0) / 2.0

        kp = self.kp_min + normalized_action[
            [0, 3]
        ] * (self.kp_max - self.kp_min)

        ki = self.ki_min + normalized_action[
            [1, 4]
        ] * (self.ki_max - self.ki_min)

        kd = self.kd_min + normalized_action[
            [2, 5]
        ] * (self.kd_max - self.kd_min)

        return (
            kp.astype(np.float32),
            ki.astype(np.float32),
            kd.astype(np.float32),
        )

    # Tạo observation cho SAC
    def build_observation(
        self,
        base_observation: np.ndarray,
    ) -> np.ndarray:
        """
        Ghép observation gốc của robot với:

            error = q_desired - q_actual

            integral_error
        """
        base_observation = np.asarray(
            base_observation,
            dtype=np.float32,
        ).reshape(-1)

        target_q = np.asarray(
            self.robot_env.target_q,
            dtype=np.float32,
        )
        current_q = np.asarray(
            self.robot_env.q,
            dtype=np.float32,
        )

        error = target_q - current_q
        observation = np.concatenate(
            [
                base_observation,
                error,
                self.integral_error,
            ]
        )
        return observation.astype(np.float32)

    # Tính torque bằng bộ điều khiển PID
    def calculate_pid_torque(self) -> np.ndarray:
        """
        Tính torque PID cho hai khớp.
        error:
            Sai số vị trí góc.
        derivative_error:
            Đạo hàm của sai số.
        Vì mục tiêu không đổi:
            derivative_error = -dq
        Torque:
            tau = Kp * error
                + Ki * integral_error
                + Kd * derivative_error
        """

        target_q = np.asarray(
            self.robot_env.target_q,
            dtype=np.float32,
        )

        current_q = np.asarray(
            self.robot_env.q,
            dtype=np.float32,
        )

        current_dq = np.asarray(
            self.robot_env.dq,
            dtype=np.float32,
        )

        error = target_q - current_q

        # Cập nhật thành phần tích phân
        self.integral_error += error * self.dt

        self.integral_error = np.clip(
            self.integral_error,
            -self.integral_limit,
            self.integral_limit,
        )

        # Đạo hàm sai số
        derivative_error = -current_dq

        # Tính torque PID
        torque = (
            self.kp * error
            + self.ki * self.integral_error
            + self.kd * derivative_error
        )

        # Giới hạn torque
        torque = np.clip(
            torque,
            -self.torque_limit,
            self.torque_limit,
        )

        return torque.astype(np.float32)

    # Reset môi trường
    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict | None = None,
    ):
        super().reset(seed=seed)

        base_observation, info = self.robot_env.reset(
            seed=seed,
            options=options,
        )

        self.step_counter = 0

        self.integral_error = np.zeros(
            2,
            dtype=np.float32,
        )

        # Đưa hệ số PID về giá trị giữa giới hạn
        self.kp = (
            self.kp_min + self.kp_max
        ) / 2.0

        self.ki = (
            self.ki_min + self.ki_max
        ) / 2.0

        self.kd = (
            self.kd_min + self.kd_max
        ) / 2.0

        observation = self.build_observation(
            base_observation
        )

        info = dict(info)

        info.update(
            {
                "kp": self.kp.copy(),
                "ki": self.ki.copy(),
                "kd": self.kd.copy(),
                "integral_error": self.integral_error.copy(),
            }
        )

        return observation, info

    # Một bước mô phỏng
    def step(
        self,
        action: np.ndarray,
    ):
        """
        Quy trình:

        1. Nhận action từ SAC.
        2. Chuyển action thành Kp, Ki, Kd.
        3. Định kỳ cập nhật hệ số PID.
        4. PID tính torque.
        5. Robot nhận torque PID.
        6. Robot trả về trạng thái mới và reward.
        """

        action = np.asarray(
            action,
            dtype=np.float32,
        )

        action = np.clip(
            action,
            -1.0,
            1.0,
        )

        # Chỉ cập nhật hệ số PID theo chu kỳ
        if (
            self.step_counter
            % self.gain_update_steps
            == 0
        ):
            (
                self.kp,
                self.ki,
                self.kd,
            ) = self.action_to_gains(action)

        # PID tự tính torque
        torque_pid = self.calculate_pid_torque()

        # Robot chỉ nhận torque từ PID
        (
            base_observation,
            reward,
            terminated,
            truncated,
            info,
        ) = self.robot_env.step(torque_pid)

        self.step_counter += 1
        self.total_time_step += 1
        if self.total_time_step%500 == 0:
            print("TỔNG SỐ TIME_STEP HIỆN TẠI: ", self.total_time_step)

        observation = self.build_observation(
            base_observation
        )

        info = dict(info)

        info.update(
            {
                "kp": self.kp.copy(),
                "ki": self.ki.copy(),
                "kd": self.kd.copy(),
                "torque_pid": torque_pid.copy(),
                "integral_error": self.integral_error.copy(),
                "step_counter": self.step_counter,
            }
        )

        return (
            observation,
            float(reward),
            bool(terminated),
            bool(truncated),
            info,
        )

    # Đóng môi trường
    def close(self):
        self.robot_env.close()

from stable_baselines3.common.callbacks import BaseCallback
class MyCallback(BaseCallback):
    def _on_step(self) -> bool:
        if self.num_timesteps % 500 == 0:
            import json
            JSON_PATH = "./Data/reward_log.json"

            with open(JSON_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)

            # Lấy dict cuối cùng trong danh sách rewar được tạo
            last_dict = data[-1]
            print("Trọng số reward trước khi sửa đổi: ", last_dict)
            

        return True




# Hàm huấn luyện SAC
def main():
    print("Creating adaptive PID SAC environment...")

    environment = AdaptivePID2REnv()

    print("Checking Gymnasium environment...")

    check_env(
        environment,
        warn=True,
    )

    environment.close()

    training_environment = Monitor(
        AdaptivePID2REnv()
    )

    print("Creating SAC model...")

    model = SAC(
        policy="MlpPolicy",
        env=training_environment,
        learning_rate=3e-4,
        buffer_size=5_000,
        learning_starts=1_000,
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        train_freq=1,
        gradient_steps=1,
        ent_coef="auto",
        verbose=1,
        tensorboard_log="./tensorboard_adaptive_pid_sac/",
        seed=42,
    )

    total_timesteps = 5_000

    print(
        f"Starting training for {total_timesteps} timesteps..."
    )

    call_back = MyCallback()
    model.learn(
        total_timesteps=total_timesteps,
        log_interval=10,
        progress_bar=True,
        callback=call_back
    )

    # Lưu mô hình
    model_directory = Path("models")

    model_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    model_path = (
        model_directory
        / "adaptive_pid_sac_2r"
    )

    model.save(model_path)

    print(
        f"Training completed. Model saved to: {model_path}"
    )

    training_environment.close()


if __name__ == "__main__":
    main()