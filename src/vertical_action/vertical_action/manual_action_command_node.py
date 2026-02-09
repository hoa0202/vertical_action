#!/usr/bin/env python3
"""
코드3: 제3의 명령 터미널.
  /action:       entering_next, entering_end
  /action_check: entering_check, goal_finish, goal_return_finish

실행:
  ros2 run vertical_action manual_action_command_node                    # 대화형
  ros2 run vertical_action manual_action_command_node entering_next      # 한 번만 전송
  ros2 run vertical_action manual_action_command_node entering_end
  ros2 run vertical_action manual_action_command_node entering_check
  ros2 run vertical_action manual_action_command_node goal_finish
  ros2 run vertical_action manual_action_command_node goal_return_finish
"""
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ManualActionCommandNode(Node):
    def __init__(self):
        super().__init__("manual_action_command_node")
        self._pub_action = self.create_publisher(String, "/action", 10)
        self._pub_action_check = self.create_publisher(String, "/action_check", 10)

    def send(self, data: str) -> None:
        msg = String()
        msg.data = data
        self._pub_action.publish(msg)
        self.get_logger().info(f"/action 발행: '{data}'")
        print(f"/action 발행: '{data}'", flush=True)

    def send_check(self, data: str) -> None:
        msg = String()
        msg.data = data
        self._pub_action_check.publish(msg)
        self.get_logger().info(f"/action_check 발행: '{data}'")
        print(f"/action_check 발행: '{data}'", flush=True)


def _stdin_ready():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def _get_key():
    if _stdin_ready():
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = ManualActionCommandNode()

    # 인자로 명령이 오면 한 번 발행 후 종료
    if len(sys.argv) > 1:
        cmd = sys.argv[1].strip()
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        if cmd in ("entering_next", "entering_end"):
            node.send(cmd)
        elif cmd in ("entering_check", "goal_finish", "goal_return_finish"):
            node.send_check(cmd)
        else:
            print(f"알 수 없는 명령: {cmd} (entering_next | entering_end | entering_check | goal_finish | goal_return_finish)", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return

    # 대화형: n/e = /action, c/r = /action_check, q = 종료
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    print("n: entering_next | e: entering_end | c: entering_check | f: goal_finish | r: goal_return_finish | q: 종료", flush=True)

    if sys.stdin.isatty():
        old_attr = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = _get_key()
                if key == "n":
                    node.send("entering_next")
                elif key == "e":
                    node.send("entering_end")
                elif key == "c":
                    node.send_check("entering_check")
                elif key == "f":
                    node.send_check("goal_finish")
                elif key == "r":
                    node.send_check("goal_return_finish")
                elif key == "q" or key == "\x03":
                    break
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    else:
        try:
            while True:
                line = input().strip().lower()
                if line in ("n", "entering_next"):
                    node.send("entering_next")
                elif line in ("e", "entering_end"):
                    node.send("entering_end")
                elif line in ("c", "entering_check"):
                    node.send_check("entering_check")
                elif line in ("f", "goal_finish"):
                    node.send_check("goal_finish")
                elif line in ("r", "goal_return_finish"):
                    node.send_check("goal_return_finish")
                elif line in ("q", "quit"):
                    break
        except (EOFError, KeyboardInterrupt):
            pass

    node.destroy_node()
    rclpy.shutdown()
    print("종료")


if __name__ == "__main__":
    main()
