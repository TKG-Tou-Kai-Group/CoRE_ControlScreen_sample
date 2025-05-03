#!/usr/bin/env python3
# filepath: /home/pi/CoRE_ControlScreen_sample/ros2_ws/src/core_control_screen/core_control_screen/screen.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from PIL import Image as PILImage
from ament_index_python.packages import get_package_share_directory
import subprocess
import re
from numba import jit

# PyOpenGLのインポート
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

TOP_VIEW_POS_X = 0.05
TOP_VIEW_POS_Y = 0.55
TOP_VIEW_SIZE = 0.4

def get_screen_resolution_xrandr():
    # xrandr の出力を取得
    output = subprocess.check_output(['xrandr', '--current'], text=True)
    # "current 1920 x 1080" のような行を正規表現で探す
    m = re.search(r'current\s+(\d+)\s+x\s+(\d+)', output)
    if m:
        return int(m.group(1)), int(m.group(2))
    raise RuntimeError("Failed to parse xrandr output")

# OpenGLテクスチャID
camera_texture = None
overlay_texture = None
top_view_texture = None

class CameraOverlayNode(Node):
    def __init__(self):
        super().__init__('camera_overlay_node')
        
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 画面サイズを先に取得
        self.screen_width, self.screen_height = get_screen_resolution_xrandr()
        
        # OpenGLの初期化
        self.init_opengl()
        
        # ROS2サブスクリプションの設定
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            sensor_qos)
        
        self.top_view_image = None
        self.top_view_subscription = self.create_subscription(
            Image,
            'top_view_image',
            self.top_view_image_callback,
            sensor_qos)
        
        self.bridge = CvBridge()
        
        # オーバーレイ画像を読み込む
        overlay_path = os.path.join(
            get_package_share_directory('core_control_screen'), "materials", "overlay.png")
        
        try:
            # オーバーレイ画像をPILで読み込み
            self.overlay_pil = PILImage.open(overlay_path).convert('RGBA')
            self.overlay_pil = self.overlay_pil.resize((self.screen_width, self.screen_height))
            
            # PILからnumpy配列に変換
            self.overlay_array = np.array(self.overlay_pil)
            
            # OpenGLテクスチャとして設定
            self.setup_overlay_texture()
            
            self.get_logger().info(f"Overlay image loaded: {overlay_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load overlay image: {e}")
            self.overlay_array = None
        
        self.top_view_pos = (
            int(self.screen_width * TOP_VIEW_POS_X), 
            int(self.screen_height * TOP_VIEW_POS_Y)
        )
        self.top_view_size = (
            int(self.screen_height * TOP_VIEW_SIZE),
            int(self.screen_height * TOP_VIEW_SIZE)
        )
        
        self.current_frame = None
        self.top_view_updated = False
        
        # グルート・メインループの設定
        glutIdleFunc(self.opengl_idle)
        
        self.get_logger().info("Camera Overlay Node initialized")

    def init_opengl(self):
        # GLUTの初期化
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE)
        glutInitWindowSize(self.screen_width, self.screen_height)
        glutCreateWindow("Camera Overlay")
        glutFullScreen()
        
        # OpenGL設定
        glEnable(GL_TEXTURE_2D)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, self.screen_width, self.screen_height, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        # テクスチャIDの生成
        global camera_texture, overlay_texture, top_view_texture
        camera_texture = glGenTextures(1)
        overlay_texture = glGenTextures(1)
        top_view_texture = glGenTextures(1)
        
        # ディスプレイコールバックの設定
        glutDisplayFunc(self.display)

    def setup_overlay_texture(self):
        if self.overlay_array is None:
            return
            
        global overlay_texture
        
        # オーバーレイ画像をテクスチャとして設定
        glBindTexture(GL_TEXTURE_2D, overlay_texture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        
        # テクスチャデータの設定（RGBAフォーマット）
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA, 
            self.screen_width, self.screen_height, 0,
            GL_RGBA, GL_UNSIGNED_BYTE, self.overlay_array
        )

    def update_camera_texture(self):
        if self.current_frame is None:
            return
            
        global camera_texture

        # 画面サイズに合わせてリサイズ
        if self.current_frame.shape[1] != self.screen_width or self.current_frame.shape[0] != self.screen_height:
            self.current_frame = cv2.resize(
                self.current_frame, 
                (self.screen_width, self.screen_height), 
                interpolation=cv2.INTER_NEAREST
            )

        # カメラフレームをテクスチャとして設定
        glBindTexture(GL_TEXTURE_2D, camera_texture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        
        # BGR -> RGB変換（OpenGLはRGBを使用）
        rgb_frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
        
        # テクスチャデータの設定
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGB, 
            self.current_frame.shape[1], self.current_frame.shape[0], 0,
            GL_RGB, GL_UNSIGNED_BYTE, rgb_frame
        )

    def update_top_view_texture(self):
        if self.top_view_image is None:
            return
            
        global top_view_texture
        
        # トップビュー画像をリサイズ
        top_view_resized = cv2.resize(
            self.top_view_image, 
            (self.top_view_size[0], self.top_view_size[1]), 
            interpolation=cv2.INTER_NEAREST
        )
        
        # BGR -> RGB変換
        rgb_top_view = cv2.cvtColor(top_view_resized, cv2.COLOR_BGR2RGB)
        
        # テクスチャとして設定
        glBindTexture(GL_TEXTURE_2D, top_view_texture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGB, 
            self.top_view_size[0], self.top_view_size[1], 0,
            GL_RGB, GL_UNSIGNED_BYTE, rgb_top_view
        )
        
        self.top_view_updated = False

    def display(self):
        # 画面をクリア
        glClear(GL_COLOR_BUFFER_BIT)
        glLoadIdentity()
        
        # カメラフレームの描画
        if self.current_frame is not None:
            self.update_camera_texture()
            
            glBindTexture(GL_TEXTURE_2D, camera_texture)
            glBegin(GL_QUADS)
            glTexCoord2f(0.0, 0.0); glVertex2f(0, 0)
            glTexCoord2f(1.0, 0.0); glVertex2f(self.screen_width, 0)
            glTexCoord2f(1.0, 1.0); glVertex2f(self.screen_width, self.screen_height)
            glTexCoord2f(0.0, 1.0); glVertex2f(0, self.screen_height)
            glEnd()
        
        # オーバーレイの描画
        if self.overlay_array is not None:
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            
            glBindTexture(GL_TEXTURE_2D, overlay_texture)
            glBegin(GL_QUADS)
            glTexCoord2f(0.0, 0.0); glVertex2f(0, 0)
            glTexCoord2f(1.0, 0.0); glVertex2f(self.screen_width, 0)
            glTexCoord2f(1.0, 1.0); glVertex2f(self.screen_width, self.screen_height)
            glTexCoord2f(0.0, 1.0); glVertex2f(0, self.screen_height)
            glEnd()
            
            glDisable(GL_BLEND)
        
        # トップビューの描画
        if self.top_view_image is not None:
            if self.top_view_updated:
                self.update_top_view_texture()
                
            glBindTexture(GL_TEXTURE_2D, top_view_texture)
            glBegin(GL_QUADS)
            glTexCoord2f(0.0, 0.0); glVertex2f(self.top_view_pos[0], self.top_view_pos[1])
            glTexCoord2f(1.0, 0.0); glVertex2f(self.top_view_pos[0] + self.top_view_size[0], self.top_view_pos[1])
            glTexCoord2f(1.0, 1.0); glVertex2f(self.top_view_pos[0] + self.top_view_size[0], self.top_view_pos[1] + self.top_view_size[1])
            glTexCoord2f(0.0, 1.0); glVertex2f(self.top_view_pos[0], self.top_view_pos[1] + self.top_view_size[1])
            glEnd()
        
        # バッファの入れ替え
        glutSwapBuffers()

    def opengl_idle(self):
        # ROSのコールバック処理を行う
        rclpy.spin_once(self, timeout_sec=0)
        
        # 画面の再描画
        if self.current_frame is not None:
            glutPostRedisplay()

    def image_callback(self, msg):
        try:
            # ROSイメージメッセージをOpenCVフォーマットに変換
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def top_view_image_callback(self, msg):
        try:
            # ROSイメージメッセージをOpenCVフォーマットに変換
            self.top_view_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.top_view_updated = True
        except Exception as e:
            self.get_logger().error(f"Error processing top view image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraOverlayNode()
    
    try:
        # OpenGLのメインループを開始
        glutMainLoop()
    except KeyboardInterrupt:
        pass
    finally:
        # クリーンアップ処理
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()