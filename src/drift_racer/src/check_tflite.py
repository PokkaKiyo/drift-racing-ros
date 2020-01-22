# converter = tf.lite.TFLiteConverter.from_keras_model_file("/content/model-05-0.126.h5")
# For image data1/554.jpeg, the prediction is [[0.08965816 0.2768537 ]]

from tflite_runtime.interpreter import Interpreter
from PIL import Image

interpreter = Interpreter("./converted_model.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

img = Image.open("./554.jpeg")
input_data = np.array(img, dtype=np.float32)
interpreter.set_tensor(input_details[0]['index'], input_data)

interpreter.invoke()

output_data = interpreter.get_tensor(output_details[0]['index'])
print(output_data)

