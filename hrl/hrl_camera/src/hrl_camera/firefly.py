import camera

class firefly(camera.camera):
    def __init__(self, camera_configuration, opencv_id):
        camera.camera.__init__(self, camera_configuration, opencv_id)
