import cv2


class Camera:
    def __init__(self, cameras=None, size=(400, 300)) -> None:
        if cameras is None:
            cameras = []
        self.camera = {}
        self.size = size
        if cameras is None:
            i = 0
            while True:
                cam = cv2.VideoCapture(i)
                if cam.read()[0]:
                    self.camera[i] = cam
                elif i > 10:
                    break
                i += 1
        else:
            self.camera = {cv2.VideoCapture(i) for i in cameras}
        print("Cams used:", self.camera.keys())

    def prepare(self, image, quality=10):
        if image is str:
            image = cv2.imread(image)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.resize(image, self.size)
        res = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), quality,
             int(cv2.IMWRITE_JPEG_OPTIMIZE), 1]
        )[1]
        return res.reshape(-1).tobytes()

    def get(self, id, quality=10):
        ret, img = self.camera[id].read()
        if ret:
            return self.prepare(img)
        else:
            return None

    def close(self):
        for _, i in self.camera.items():
            i.release()
