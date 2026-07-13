import numpy as np

# Limitation
def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def map_range(x, in_min, in_max, out_min=0, out_max=255, reverse=False):
    x = clamp(float(x), in_min, in_max)
    t = (x - in_min) / (in_max - in_min + 1e-8)
    if reverse:
        t = 1.0 - t
    return int(round(out_min + t * (out_max - out_min)))


class L20Bridge:
    """

    输入 joint_names:
      thumb_cmc_yaw
      thumb_cmc_pitch
      thumb_cmc_roll
      thumb_mcp
      index_mcp_roll
      index_mcp_pitch
      index_pip
      middle_mcp_roll
      middle_mcp_pitch
      middle_pip
      ring_mcp_roll
      ring_mcp_pitch
      ring_pip
      pinky_mcp_roll
      pinky_mcp_pitch
      pinky_pip

    输出 L20 20维命令:
      0 拇指根部
      1 食指根部
      2 中指根部
      3 无名指根部
      4 小指根部
      5 拇指侧摆
      6 食指侧摆
      7 中指侧摆
      8 无名指侧摆
      9 小指侧摆
      10 拇指横摆
      11~14 预留
      15 拇指尖部
      16 食指末端
      17 中指末端
      18 无名指末端
      19 小指末端
    """

    EXPECTED_JOINTS = [
        "thumb_cmc_yaw",
        "thumb_cmc_pitch",
        "thumb_cmc_roll",
        "thumb_mcp",
        "index_mcp_roll",
        "index_mcp_pitch",
        "index_pip",
        "middle_mcp_roll",
        "middle_mcp_pitch",
        "middle_pip",
        "ring_mcp_roll",
        "ring_mcp_pitch",
        "ring_pip",
        "pinky_mcp_roll",
        "pinky_mcp_pitch",
        "pinky_pip",
    ]

    def __init__(self, alpha=0.2, verbose=False):
        self.alpha = alpha
        self.verbose = verbose # Print info?
        self.default_cmd = 200
        self.prev_cmd = np.ones(20, dtype=np.float32) * self.default_cmd

    
        # 例如设成 1 就只测食指根部，设成 16 就只测食指末端
        self.debug_single_slot = None


        self.limits = {
            "thumb_cmc_yaw": (0.6, 1.3, False), # 拇指侧摆 #
            "thumb_cmc_pitch": (0.0, 0.5, False), # 拇指根部
            "thumb_cmc_roll": (0.0, 0.7, False), # 拇指横摆 #
            "thumb_mcp": (0.0, 0.4, False), # 拇指尖部 #

            "index_mcp_roll": (-0.2, 0.2, False), #
            "index_mcp_pitch": (0.0, 1.2, False),#
            "index_pip": (0.0, 1.1, False), #

            "middle_mcp_roll": (-0.2, 0.2, False),#
            "middle_mcp_pitch": (0.0, 1.4, False),#
            "middle_pip": (0.0, 1.4, False), #

            "ring_mcp_roll": (-0.2, 0.2, False),#
            "ring_mcp_pitch": (0.0, 1.4, False),#
            "ring_pip": (0.0, 1.5, False),#

            "pinky_mcp_roll": (-0.2, 0.2, False),#
            "pinky_mcp_pitch": (0.0, 0.6, False),#
            "pinky_pip": (1.1, 1.5, False), #
        }

    def _q(self, name_to_q, name, default=0.0):
        return float(name_to_q.get(name, default))

    # Convert!!!!!!! 
    def convert(self, joint_names, qpos):
        if len(joint_names) != len(qpos):
            raise ValueError(
                f"joint_names 和 qpos 长度不一致: {len(joint_names)} vs {len(qpos)}"
            )

        name_to_q = {name: float(qpos[i]) for i, name in enumerate(joint_names)}
        cmd = np.ones(20, dtype=np.float32) * self.default_cmd

        # 根部
        cmd[0] = map_range(self._q(name_to_q, "thumb_cmc_pitch"), *self.limits["thumb_cmc_pitch"])
        
        # input_val = self._q(name_to_q, "index_mcp_pitch")
        # in_min, in_max, reverse_flag = self.limits["index_mcp_pitch"]
        # print(f"正在映射 'index_mcp_pitch':")
        # print(f"  输入值 (x): {input_val}")
        # print(f"  输入范围: ({in_min}, {in_max})")
        # print(f"  是否反转 (reverse): {reverse_flag}")

        cmd[1] = map_range(self._q(name_to_q, "index_mcp_pitch"),*self.limits["index_mcp_pitch"][:2],0,255,True)
        # cmd[1] = map_range(self._q(name_to_q, "index_mcp_pitch"), *self.limits["index_mcp_pitch"])
        cmd[2] = map_range(self._q(name_to_q, "middle_mcp_pitch"), *self.limits["middle_mcp_pitch"][:2],0,255,True)
        cmd[3] = map_range(self._q(name_to_q, "ring_mcp_pitch"), *self.limits["ring_mcp_pitch"][:2],0,255,True)
        cmd[4] = map_range(self._q(name_to_q, "pinky_mcp_pitch"), *self.limits["pinky_mcp_pitch"][:2],0,255,True)

        # 侧摆
        cmd[5] = map_range(self._q(name_to_q, "thumb_cmc_yaw"), *self.limits["thumb_cmc_yaw"][:2],0,255,True)
        cmd[6] = map_range(self._q(name_to_q, "index_mcp_roll"), *self.limits["index_mcp_roll"][:2],0,255,True)
        cmd[7] = map_range(self._q(name_to_q, "middle_mcp_roll"), *self.limits["middle_mcp_roll"][:2],0,255,True)
        cmd[8] = map_range(self._q(name_to_q, "ring_mcp_roll"), *self.limits["ring_mcp_roll"][:2],0,255,True)
        cmd[9] = map_range(self._q(name_to_q, "pinky_mcp_roll"), *self.limits["pinky_mcp_roll"][:2],0,255,True)

        # 拇指横摆
        cmd[10] = map_range(self._q(name_to_q, "thumb_cmc_roll"), *self.limits["thumb_cmc_roll"][:2],0,255,True)

        # 末端
        cmd[15] = map_range(self._q(name_to_q, "thumb_mcp"), *self.limits["thumb_mcp"][:2],0,255,True)
        cmd[16] = map_range(self._q(name_to_q, "index_pip"), *self.limits["index_pip"][:2],0,255,True)
        cmd[17] = map_range(self._q(name_to_q, "middle_pip"), *self.limits["middle_pip"][:2],0,255,True)
        cmd[18] = map_range(self._q(name_to_q, "ring_pip"), *self.limits["ring_pip"][:2],0,255,True)
        cmd[19] = map_range(self._q(name_to_q, "pinky_pip"), *self.limits["pinky_pip"][:2],0,255,True)

        # 预留位固定中位
        cmd[11] = self.default_cmd
        cmd[12] = self.default_cmd
        cmd[13] = self.default_cmd
        cmd[14] = self.default_cmd

        # low-pass filter
        cmd = self.alpha * cmd + (1.0 - self.alpha) * self.prev_cmd
        self.prev_cmd = cmd.copy()
        # print(cmd)

        # Convert to non-float
        out = np.round(cmd).astype(np.int32).tolist()

        # 单槽位调试模式
        if self.debug_single_slot is not None:
            single = [255] * 20
            single[self.debug_single_slot] = out[self.debug_single_slot]
            out = single

        # Print debug info
        if self.verbose:
            print("[L20Bridge] joint_names =", joint_names)
            print("[L20Bridge] qpos =", np.round(qpos, 4).tolist())
            print("[L20Bridge] cmd =", out)

        return out




