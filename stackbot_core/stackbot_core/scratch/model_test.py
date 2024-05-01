import numpy as np
import tflite_runtime.interpreter as tflite
import cv2
import time

# Load the TensorFlow Lite model.
interpreter = tflite.Interpreter(model_path="/home/STACKBOT/stackbot_ws/models/MobnetV3Small_30thApril_last8Trainable_1extraHidLayer1.tflite")
interpreter.allocate_tensors()

# Get input and output details.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Function to preprocess input image
def preprocess_image(image_path, target_size=(224, 224)):
    img = cv2.imread(image_path)
    img = cv2.resize(img, target_size)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convert image to RGB format
    img = img.astype(np.float32) / 255.0  # Normalize pixel values to [0, 1]
    return img


image_path = '/home/STACKBOT/stackbot_ws/src/stackbot_core/stackbot_core/scratch/captured_image.jpg'  # Path to your image
input_image = preprocess_image(image_path)
input_image = np.expand_dims(input_image, axis=0)  # Add batch dimension

# Set input tensor.
interpreter.set_tensor(input_details[0]['index'], input_image)
start = time.time()
# Run inference.
interpreter.invoke()

# Get output tensor.
output_data = interpreter.get_tensor(output_details[0]['index'])
print(time.time() - start)
print(output_data)
