import cv2
import pyzed.sl as sl
import ogl_viewer.viewer as gl
# viewer = gl.Viewer()
viewer = gl.GLViewer()
# viewer.init(camera_infos.camera_configuration.calibration_parameters.left_cam, has_imu)
cap = cv2.VideoCapture(0)
zed = sl.Camera()
def loop():
    check,frame = cap.read()
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    if check:
        viewer.set_image(frame)
