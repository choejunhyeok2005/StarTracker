"""
services/web_server.py

Flask MJPEG Streaming Server

담당 기능:
- 웹 기반 실시간 카메라 피드 스트리밍
- MJPEG 형식으로 프레임 전송
"""

from flask import Flask, Response

import config


def create_streaming_app(camera_manager):
    """
    카메라 매니저를 이용한 Flask MJPEG 앱 생성.

    Parameters
    ----------
    camera_manager : CameraManager
        스트리밍용 카메라 매니저 인스턴스

    Returns
    -------
    Flask
        스트리밍 앱
    """

    app = Flask(__name__)

    @app.route("/")
    def video_feed():
        """MJPEG 스트림 제공."""
        return Response(
            camera_manager.generate_frames(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    return app
