import numpy as np
import io
import picamera
import time

from PIL import Image
from tflite_runtime.interpreter import Interpreter

class BehaviouralCloningAgent:
    def __init__(self):
        self._interpreter = Interpreter("./converted_model.tflite")
        self._interpreter.allocate_tensors()

        print(self._interpreter.get_input_details())
        print(self._interpreter.get_output_details())
        _, self._input_height, self._input_width, _ = self._interpreter.get_input_details()[0]['shape']
        print(self._input_height)
        print(self._input_width)

        self._socket = None # TODO

        self.main()

    def main(self):
        input_details = self._interpreter.get_input_details()
        output_details = self._interpreter.get_output_details()
        with picamera.PiCamera(resolution=(1296, 730), framerate=30) as camera:
            # camera.vflip = True
            camera.start_preview()
            try:
                stream = io.BytesIO()
                for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
                    stream.seek(0)
                    image = Image.open(stream).convert('RGB').resize((self._input_width, self._input_height), Image.ANTIALIAS)
                    start_time = time.time()

                    img = np.asarray(image)
                    img = img[np.newaxis, ...]
                    input_data = np.array(img, dtype=np.float32)
                    self._interpreter.set_tensor(input_details[0]['index'], input_data)

                    self._interpreter.invoke()
                    output_data = self._interpreter.get_tensor(output_details[0]['index'])[0]
                    time_taken_ms = (time.time() - start_time) * 1000
                    print(f'output_data:{output_data}, time_taken:{time_taken_ms}ms')
                    stream.seek(0)
                    stream.truncate()
                    camera.annotate_text = str(output_data) + ", " + str(time_taken_ms)
            except KeyboardInterrupt:
                print("BehaviouralCloningAgent: Ctrl-C")
            finally:
                camera.stop_preview()
                print("BehaviouralCloningAgent: done")

if __name__ == "__main__":
    BehaviouralCloningAgent()
