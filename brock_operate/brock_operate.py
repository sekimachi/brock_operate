import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from imrc_messages.srv import BrockOperate


class BrockOperateNode(Node):

    def __init__(self):
        super().__init__("brock_operate")

        # =========================================================
        # パラメータ
        # =========================================================

        # 画面中心
        self.CENTER_X = 960.0

        # 中心判定範囲
        # 940 <= center_x <= 980
        self.CENTER_TOLERANCE = 20.0

        # =========================================================
        # 回転速度 P制御
        # =========================================================

        # 最小回転速度
        self.ROTATE_MIN = 0.1

        # 最大回転速度
        self.ROTATE_MAX = 0.5

        # Pゲイン
        self.KP = 0.001

        # =========================================================
        # brocks_info
        # =========================================================

        # 1段目が見えているか
        self.first_brock_visible = False

        # 1段目のcenter_x
        self.first_center_x = 0.0

        # 1段目のdistance
        self.first_distance = 0.0

        # =========================================================
        # Service状態
        # =========================================================

        self.service_active = False

        # 現在処理している色
        self.current_color = None

        # =========================================================
        # Service結果
        # =========================================================

        self.service_success = False
        self.service_distance = 0.0

        # =========================================================
        # Lock
        # =========================================================

        self.state_lock = threading.Lock()

        # =========================================================
        # Service完了通知
        # =========================================================

        self.service_done = threading.Event()

        # =========================================================
        # Callback Group
        # =========================================================

        self.callback_group = ReentrantCallbackGroup()

        # =========================================================
        # brocks_info Subscriber
        # =========================================================

        self.subscription = self.create_subscription(
            String,
            "brocks_info",
            self.on_brocks_info,
            10,
            callback_group=self.callback_group
        )

        # =========================================================
        # brocks_info Publisher
        #
        # Service終了後に情報をリセットするために使用
        # =========================================================

        self.brocks_info_publisher = self.create_publisher(
            String,
            "brocks_info",
            10
        )

        # =========================================================
        # brock_color Publisher
        # =========================================================

        self.brock_color_publisher = self.create_publisher(
            String,
            "brock_color",
            10
        )

        # =========================================================
        # cmd_vel_brock Publisher
        # =========================================================

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "cmd_vel_brock",
            10
        )

        # =========================================================
        # Service
        # =========================================================

        self.service = self.create_service(
            BrockOperate,
            "brock_operate",
            self.on_brock_operate,
            callback_group=self.callback_group
        )

        # =========================================================
        # 制御ループ
        # =========================================================

        self.timer = self.create_timer(
            0.01,
            self.control_loop,
            callback_group=self.callback_group
        )

        # =========================================================
        # 起動ログ
        # =========================================================

        self.get_logger().info(
            "brock_operate started"
        )

        self.get_logger().info(
            f"P制御: KP={self.KP}, "
            f"ROTATE_MIN={self.ROTATE_MIN}, "
            f"ROTATE_MAX={self.ROTATE_MAX}"
        )

    # =============================================================
    # brocks_info受信
    # =============================================================

    def on_brocks_info(self, msg):

        lines = [
            line.strip()
            for line in msg.data.split("\n")
            if line.strip()
        ]

        # =========================================================
        # 1段目がない
        # =========================================================

        if not lines:

            with self.state_lock:

                self.first_brock_visible = False
                self.first_center_x = 0.0
                self.first_distance = 0.0

            return

        # =========================================================
        # 1段目
        # =========================================================

        first_line = lines[0]

        center_x = None
        distance = None

        # =========================================================
        # 情報を解析
        # =========================================================

        for item in first_line.split(","):

            item = item.strip()

            # -----------------------------------------------------
            # distance
            # -----------------------------------------------------

            if item.startswith("distance="):

                try:

                    distance = float(
                        item.split("=", 1)[1]
                    )

                except ValueError:

                    pass

            # -----------------------------------------------------
            # center_x
            # -----------------------------------------------------

            elif item.startswith("center_x="):

                try:

                    center_x = float(
                        item.split("=", 1)[1]
                    )

                except ValueError:

                    pass

        # =========================================================
        # 状態更新
        # =========================================================

        with self.state_lock:

            self.first_brock_visible = True

            if center_x is not None:
                self.first_center_x = center_x

            if distance is not None:
                self.first_distance = distance

    # =============================================================
    # Service callback
    # =============================================================

    def on_brock_operate(self, request, response):

        color = request.color.strip().lower()

        self.get_logger().info(
            f"Service received: {color}"
        )

        # =========================================================
        # red / blue以外
        # =========================================================

        if color not in ["red", "blue"]:

            response.success = False
            response.distance = 0.0

            self.get_logger().warning(
                f"Invalid color: {color}"
            )

            return response

        # =========================================================
        # すでにService処理中
        # =========================================================

        with self.state_lock:

            if self.service_active:

                response.success = False
                response.distance = 0.0

                self.get_logger().warning(
                    "Service is already active"
                )

                return response

            # -----------------------------------------------------
            # Service開始
            # -----------------------------------------------------

            self.service_active = True
            self.current_color = color

            self.service_success = False
            self.service_distance = 0.0

            # -----------------------------------------------------
            # 前回の検出情報をリセット
            #
            # これが重要
            #
            # 前回center_x=320が残っていると、
            # 2回目のServiceが即成功してしまうため。
            # -----------------------------------------------------

            self.first_brock_visible = False
            self.first_center_x = 0.0
            self.first_distance = 0.0

            # -----------------------------------------------------
            # 完了Eventをリセット
            # -----------------------------------------------------

            self.service_done.clear()

        # =========================================================
        # 逆色を決定
        # =========================================================

        if color == "red":
            opposite_color = "blue"
        else:
            opposite_color = "red"

        # =========================================================
        # brock_colorへ送信
        # =========================================================

        color_msg = String()
        color_msg.data = opposite_color

        self.brock_color_publisher.publish(
            color_msg
        )

        self.get_logger().info(
            f"/brock_color -> {opposite_color}"
        )

        # =========================================================
        # 位置合わせ開始
        # =========================================================

        self.get_logger().info(
            "位置合わせを開始します"
        )

        # =========================================================
        # 中央に到達するまで待つ
        # =========================================================

        while rclpy.ok():

            if self.service_done.wait(
                timeout=0.1
            ):
                break

        # =========================================================
        # 結果をResponseへ設定
        # =========================================================

        with self.state_lock:

            response.success = self.service_success
            response.distance = self.service_distance

            result_distance = self.service_distance

            self.service_active = False
            self.current_color = None

        # =========================================================
        # 停止
        # =========================================================

        self.publish_stop()

        # =========================================================
        # ねんのためbrocks_infoをリセット
        # =========================================================

        reset_msg = String()

        reset_msg.data = (
            "distance=0.00,"
            "tier=1,"
            "model=red,"
            "height=150,"
            "center_x=0"
        )

        self.brocks_info_publisher.publish(
            reset_msg
        )

        self.get_logger().info(
            f"/brocks_info -> {reset_msg.data}"
        )

        # =========================================================
        # 結果ログ
        # =========================================================

        self.get_logger().info(
            f"Service Response: "
            f"success={response.success}, "
            f"distance={result_distance:.3f}"
        )

        return response

    # =============================================================
    # 制御ループ
    # =============================================================

    def control_loop(self):

        twist = Twist()

        # =========================================================
        # 状態取得
        # =========================================================

        with self.state_lock:

            active = self.service_active
            visible = self.first_brock_visible
            center_x = self.first_center_x
            distance = self.first_distance

        # =========================================================
        # Serviceを受け取っていない
        # =========================================================

        if not active:

            self.publish_stop(twist)

            return

        # =========================================================
        # 1段目が見えていない
        # =========================================================

        if not visible:

            # -----------------------------------------------------
            # 常に右回転
            # -----------------------------------------------------

            twist.angular.z = -self.ROTATE_MAX

            self.cmd_vel_publisher.publish(
                twist
            )

            self.get_logger().info(
                "1段目なし -> 右回転"
            )

            return

        # =========================================================
        # 1段目が見えている
        # =========================================================

        self.get_logger().info(
            f"1段目: "
            f"center_x={center_x:.1f}, "
            f"distance={distance:.3f}"
        )

        # =========================================================
        # 中心からのズレ
        # =========================================================

        dx = center_x - self.CENTER_X

        # =========================================================
        # 中央判定
        #
        # 620 <= center_x <= 660
        # =========================================================

        if abs(dx) <= self.CENTER_TOLERANCE:

            # -----------------------------------------------------
            # 停止
            # -----------------------------------------------------

            self.publish_stop(twist)

            self.get_logger().info(
                f"中央到達: center_x={center_x:.1f}"
            )

            self.get_logger().info(
                f"distance={distance:.3f}"
            )

            # -----------------------------------------------------
            # Service結果を保存
            # -----------------------------------------------------

            with self.state_lock:

                self.service_success = True
                self.service_distance = distance

                # 制御停止
                self.service_active = False
                self.current_color = None

            # -----------------------------------------------------
            # Service callbackへ完了通知
            # -----------------------------------------------------

            self.service_done.set()

            self.get_logger().info(
                "Service success=True"
            )

            self.get_logger().info(
                f"Service distance={distance:.3f}"
            )

            return

        # =========================================================
        # P制御による回転速度計算
        # =========================================================

        rotate_vel = abs(dx) * self.KP

        # =========================================================
        # 回転速度を0.1～0.5に制限
        # =========================================================

        rotate_vel = max(
            self.ROTATE_MIN,
            min(rotate_vel, self.ROTATE_MAX)
        )

        # =========================================================
        # ブロックが左
        #
        # center_x < CENTER_X
        # =========================================================

        if center_x < self.CENTER_X:

            twist.angular.z = rotate_vel

            self.cmd_vel_publisher.publish(
                twist
            )

            self.get_logger().info(
                f"左回転: "
                f"center_x={center_x:.1f}, "
                f"dx={dx:.1f}, "
                f"rotate_vel={rotate_vel:.3f}"
            )

            return

        # =========================================================
        # ブロックが右
        #
        # center_x > CENTER_X
        # =========================================================

        if center_x > self.CENTER_X:

            twist.angular.z = -rotate_vel

            self.cmd_vel_publisher.publish(
                twist
            )

            self.get_logger().info(
                f"右回転: "
                f"center_x={center_x:.1f}, "
                f"dx={dx:.1f}, "
                f"rotate_vel={rotate_vel:.3f}"
            )

            return

    # =============================================================
    # 停止
    # =============================================================

    def publish_stop(self, twist=None):

        if twist is None:
            twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(
            twist
        )


# =============================================================
# main
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = BrockOperateNode()

    # =========================================================
    # MultiThreadedExecutor
    # =========================================================

    executor = MultiThreadedExecutor(
        num_threads=3
    )

    executor.add_node(node)

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        # -----------------------------------------------------
        # 終了時停止
        # -----------------------------------------------------

        node.publish_stop()

        executor.shutdown()

        node.destroy_node()

        rclpy.shutdown()


# =============================================================
# Entry Point
# =============================================================

if __name__ == "__main__":
    main()