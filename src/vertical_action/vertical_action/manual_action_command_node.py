#!/usr/bin/env python3
"""
코드3: 제3의 명령 터미널.
goal_finish 확인 후 /action 토픽으로 수동 전송:
  entering_next : 다음 단계로
  entering_end  : 시나리오 종료

실행:
  ros2 run vertical_action manual_action_command_node              # 대화형 (n/e/q)
  ros2 run vertical_action manual_action_command_node entering_next # 한 번만 전송 후 종료
  ros2 run vertical_action manual_action_command_node entering_end
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
        self._pub = self.create_publisher(String, "/action", 10)
        self._action_topic = "/action"

    def send(self, data: str) -> None:
        msg = String()
        msg.data = data
        self._pub.publish(msg)
        self.get_logger().info(f"/action 발행: '{data}'")
        print(f"/action 발행: '{data}'", flush=True)


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
        if cmd in ("entering_next", "entering_end"):
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
            node.send(cmd)
            node.destroy_node()
            rclpy.shutdown()
            return
        print(f"알 수 없는 명령: {cmd} (entering_next | entering_end)", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return

    # 대화형: n = entering_next, e = entering_end, q = 종료
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    print("n: entering_next | e: entering_end | q: 종료", flush=True)

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
                elif line in ("q", "quit"):
                    break
        except (EOFError, KeyboardInterrupt):
            pass

    node.destroy_node()
    rclpy.shutdown()
    print("종료")


if __name__ == "__main__":
    main()
