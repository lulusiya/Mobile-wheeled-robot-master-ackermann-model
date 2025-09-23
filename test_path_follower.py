#!/usr/bin/env python3

"""
简单的测试脚本，用于验证path_follower是否可以正常启动
"""

import subprocess
import sys
import os
import time


def test_path_follower():
    print("=== 测试 path_follower.py 脚本 ===")

    # 检查文件是否存在
    script_path = "src/simple_robot/scripts/path_follower.py"
    if not os.path.exists(script_path):
        print(f"错误: {script_path} 不存在")
        return False

    print(f"✓ 脚本文件存在: {script_path}")

    # 检查path.txt是否存在
    path_file = "path.txt"
    if not os.path.exists(path_file):
        print(f"错误: {path_file} 不存在")
        return False

    print(f"✓ 路径文件存在: {path_file}")

    # 检查脚本权限
    if os.access(script_path, os.X_OK):
        print("✓ 脚本有执行权限")
    else:
        print("! 脚本没有执行权限，尝试添加...")
        os.chmod(script_path, 0o755)

    # 检查shebang行
    with open(script_path, "r", encoding="utf-8") as f:
        first_line = f.readline().strip()
        if first_line == "#!/usr/bin/env python3":
            print("✓ Shebang行正确")
        else:
            print(f"! Shebang行可能有问题: {repr(first_line)}")

    print("\n=== 测试完成 ===")
    return True


if __name__ == "__main__":
    success = test_path_follower()
    if success:
        print("\n所有检查通过！现在可以尝试启动仿真了。")
    else:
        print("\n发现问题，请修复后再试。")
