# for ROS2
import rclpy
from rclpy.node import Node

# from cam_msg.msg import CamJpeg
from cam_msg.msg import CamJpeg

# for WebAPI
from flask import Flask, jsonify, request
import base64

# for integration flask and WebAPI
import threading

# for argument
import sys

# for type hint
from typing import Optional

# for checking webapi request
lock_camdata: threading.Lock = threading.Lock()
camera_data: Optional[bytes] = None
occured_date: Optional[str] = None


# ROS Subscriber
class CamSubscriber(Node):
    def __init__(self):
        super().__init__(sys.argv[1])
        self.subscription = self.create_subscription(
            CamJpeg, sys.argv[2], self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: CamJpeg):
        global camera_data, occured_date
        lock_camdata.acquire()
        camera_data = msg.unbounded_cam_data
        occured_date = msg.yyyymmdd_hhmmss_string
        lock_camdata.release()
        print(f"Received {occured_date} data.")


# Flask thread
def flask_func():
    # create app object
    app = Flask(__name__)

    # define flask api
    @app.route("/GET_LIVINGPIC", methods=["GET"])
    def get_camdata_api():
        global camera_data, occured_date
        lock_camdata.acquire()
        if camera_data != None and occured_date != None:
            print(f"Start convert {occured_date} data.")
            # convert bytes list to int list
            camdata_data_ints = [int.from_bytes(b, "big") for b in camera_data]
            # convert int list to bytes
            camdata_data_bytes = bytes(camdata_data_ints)
            # convert bytes to bytes(base64 format)
            camdata_data_b64 = base64.b64encode(camdata_data_bytes)
            # convert bytes(base64 format) to string
            camdata_data_str = camdata_data_b64.decode()

            if camdata_data_b64 != None:
                print(f"Start send {occured_date} data.")
                lock_camdata.release()
                return (
                    jsonify(
                        {"image_data": camdata_data_str, "occured_date": occured_date}
                    ),
                    200,
                )
        # Error case
        lock_camdata.release()
        return (
            jsonify({}),
            500,
        )

    # execute flask
    app.run(debug=False, host="0.0.0.0")


def main(args=None):
    # check argument
    if len(sys.argv) == 3:
        args = sys.argv[1:]
    else:
        print("You shoud give two arguments.")
        print(f"{sys.argv[0]} [node_name] [topic_name]")
        exit(1)

    # start flask thread
    flask_thread = threading.Thread(target=flask_func)
    flask_thread.start()

    # init ROS2
    rclpy.init(args=args)
    cam_subscriber = CamSubscriber()

    # start ROS2 subscriber
    rclpy.spin(cam_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cam_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
