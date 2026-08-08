import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from imrc_messages.srv import BrockOperate


class BrockOperateNode(Node):

    def __init__(self):
        super().__init__("brock_operate")

        # =========================================================
        # パラメータ
        # =========================================================

        # カメラ画像幅
        self.IMAGE_WIDTH = 640

        # 画面中心
        self.CENTER_X = 320.0

        # 中心判定範囲
        # 320 ± 10
        self.CENTER_TOLERANCE = 10.0

        # 回転速度
        self.ROTATE_VEL = 0.1

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

        # Service処理中か
        self.service_active = False

        # 現在処理している色
        self.current_color = None

        # Service処理用Lock
        self.service_lock = threading.Lock()

        # =========================================================
        # Service完了通知
        # =========================================================

        self.service_done = threading.Event()

        # =========================================================
        # Service Responseを保持
        # =========================================================

        self.service_response = None

        # =========================================================
        # brocks_info Subscriber
        # =========================================================

        self.subscription = self.create_subscription(
            String,
            "brocks_info",
            self.on_brocks_info,
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
            self.on_brock_operate
        )

        # =========================================================
        # 制御ループ
        # =========================================================

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # =========================================================
        # 起動ログ
        # =========================================================

        self.get_logger().info(
            "brock_operate started"
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
        # 1段目なし
        # =========================================================

        if not lines:

            self.first_brock_visible = False
            self.first_center_x = 0.0
            self.first_distance = 0.0

            return

        # =========================================================
        # 1段目あり
        # =========================================================

        self.first_brock_visible = True

        # 1段目
        first_line = lines[0]

        # =========================================================
        # 各情報を取得
        # =========================================================

        for item in first_line.split(","):

            item = item.strip()

            # -----------------------------------------------------
            # distance
            # -----------------------------------------------------

            if item.startswith("distance="):

                try:

                    self.first_distance = float(
                        item.split("=", 1)[1]
                    )

                except ValueError:

                    self.first_distance = 0.0

            # -----------------------------------------------------
            # center_x
            # -----------------------------------------------------

            elif item.startswith("center_x="):

                try:

                    self.first_center_x = float(
                        item.split("=", 1)[1]
                    )

                except ValueError:

                    self.first_center_x = 0.0

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
        # Serviceがすでに動作中
        # =========================================================

        with self.service_lock:

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

            # Responseを保存
            self.service_response = response

        # =========================================================
        # 逆色を送信
        # =========================================================

        if color == "red":

            opposite_color = "blue"

        else:

            opposite_color = "red"

        color_msg = String()

        color_msg.data = opposite_color

        self.brock_color_publisher.publish(
            color_msg
        )

        self.get_logger().info(
            f"/brock_color -> {opposite_color}"
        )

        # =========================================================
        # ここでは待たない
        #
        # control_loop() がこれ以降の動作を担当する
        # =========================================================

        self.get_logger().info(
            "位置合わせを開始します"
        )

        # =========================================================
        # Service callback終了
        #
        # 現時点ではResponseはまだ完成していない
        # =========================================================

        return response

    # =============================================================
    # 制御ループ
    # =============================================================

    def control_loop(self):

        twist = Twist()

        # =========================================================
        # Serviceを受け取っていない
        # =========================================================

        if not self.service_active:

            self.publish_stop(twist)

            return

        # =========================================================
        # 1段目が見えていない
        # =========================================================

        if not self.first_brock_visible:

            # -----------------------------------------------------
            # 常に右回転
            # -----------------------------------------------------

            twist.angular.z = -self.ROTATE_VEL

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

        center_x = self.first_center_x
        distance = self.first_distance

        self.get_logger().info(
            f"1段目: center_x={center_x:.1f}, "
            f"distance={distance:.3f}"
        )

        # =========================================================
        # 中央判定
        # =========================================================

        dx = center_x - self.CENTER_X

        if abs(dx) <= self.CENTER_TOLERANCE:

            # -----------------------------------------------------
            # 中央に到達
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

            if self.service_response is not None:

                self.service_response.success = True
                self.service_response.distance = distance

            # -----------------------------------------------------
            # Service終了
            # -----------------------------------------------------

            self.service_active = False

            self.current_color = None

            # -----------------------------------------------------
            # 結果をログ表示
            # -----------------------------------------------------

            self.get_logger().info(
                "Service success=True"
            )

            self.get_logger().info(
                f"Service distance={distance:.3f}"
            )

            return

        # =========================================================
        # ブロックが左
        # =========================================================

        if center_x < self.CENTER_X:

            twist.angular.z = self.ROTATE_VEL

            self.cmd_vel_publisher.publish(
                twist
            )

            self.get_logger().info(
                f"左回転: center_x={center_x:.1f}"
            )

            return

        # =========================================================
        # ブロックが右
        # =========================================================

        if center_x > self.CENTER_X:

            twist.angular.z = -self.ROTATE_VEL

            self.cmd_vel_publisher.publish(
                twist
            )

            self.get_logger().info(
                f"右回転: center_x={center_x:.1f}"
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