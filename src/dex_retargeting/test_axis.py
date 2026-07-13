from pathlib import Path
import numpy as np
import sapien
from sapien.utils import Viewer

from dex_retargeting.retargeting_config import RetargetingConfig

# 1) 指到你的 hand 资产根目录
# 这个目录下面应该能找到 linkerhand_l20/left/linkerhand_l20_left.urdf
HAND_ROOT = "/home/xueliang/dex-retargeting/assets/robots/hands"

# 2) 你的配置文件
CFG_PATH = "/home/xueliang/dex-retargeting/src/dex_retargeting/configs/teleop/linkerhand_l20_left.yml"


def main():
    RetargetingConfig.set_default_urdf_dir(HAND_ROOT)
    config = RetargetingConfig.load_from_file(CFG_PATH)

    sapien.render.set_viewer_shader_dir("default")
    sapien.render.set_camera_shader_dir("default")

    scene = sapien.Scene()
    scene.add_ground(-0.2)

    scene.add_directional_light(np.array([1, 1, -1]), np.array([3, 3, 3]))
    scene.add_point_light(np.array([2, 2, 2]), np.array([2, 2, 2]), shadow=False)
    scene.add_point_light(np.array([2, -2, 2]), np.array([2, 2, 2]), shadow=False)

    viewer = Viewer()
    viewer.set_scene(scene)
    viewer.control_window.show_origin_frame = True
    viewer.control_window.move_speed = 0.02

    loader = scene.create_urdf_loader()
    loader.load_multiple_collisions_from_file = True

    # 直接加载你配置里写的 URDF
    urdf_path = Path(config.urdf_path)
    robot = loader.load(str(urdf_path))
    robot.set_pose(sapien.Pose([0, 0, 0]))

    active_joints = robot.get_active_joints()
    joint_names = [j.get_name() for j in active_joints]

    print("Active joints:")
    for i, name in enumerate(joint_names):
        print(f"{i:2d}: {name}")

    target_name = "pinky_mcp_pitch"
    assert target_name in joint_names, f"{target_name} not found in active joints"

    target_idx = joint_names.index(target_name)
    qpos = np.zeros(len(active_joints), dtype=np.float32)

    print("\nControls:")
    print(" j: pinky_mcp_roll += 0.05")
    print(" k: pinky_mcp_roll -= 0.05")
    print(" r: reset all joints to 0")
    print(" q: quit")

    while not viewer.closed:
        robot.set_qpos(qpos)
        scene.update_render()
        viewer.render()

        key = viewer.window.key_down
        if key("j"):
            qpos[target_idx] += 0.05
            print(f"{target_name} = {qpos[target_idx]:.3f}")
        elif key("k"):
            qpos[target_idx] -= 0.05
            print(f"{target_name} = {qpos[target_idx]:.3f}")
        elif key("r"):
            qpos[:] = 0
            print("reset qpos")
        elif key("q"):
            break


if __name__ == "__main__":
    main()




