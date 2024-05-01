import numpy as np
import tflite_runtime.interpreter as tflite
import cv2
import time
import rclpy
from geometry_msgs.msg import Twist

# Initialize ROS 2 node
rclpy.init()
node = rclpy.create_node('inference_publisher')

# Create a publisher for Twist messages
publisher = node.create_publisher(Twist, '/cmd_vel', 10)

# Load the TensorFlow Lite model.
interpreter = tflite.Interpreter(model_path="/home/STACKBOT/stackbot_ws/models/MobnetV3Small_30thApril_last14Trainable_1extraHidLayer1_250Epoch.tflite")
interpreter.allocate_tensors()

# Get input and output details.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Function to preprocess input image
def preprocess_image(image, target_size=(224, 224)):
    img = cv2.resize(image, target_size)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convert image to RGB format
    img = img.astype(np.float32) / 255.0  # Normalize pixel values to [0, 1]
    return img

# Function to perform inference on a single frame
def inference(image):
    input_image = preprocess_image(image)
    input_image = np.expand_dims(input_image, axis=0)  # Add batch dimension

    # Set input tensor.
    interpreter.set_tensor(input_details[0]['index'], input_image)

    # Run inference.
    interpreter.invoke()

    # Get output tensor.
    output_data = interpreter.get_tensor(output_details[0]['index'])
    return output_data

# Initialize the camera capture
capture = cv2.VideoCapture(0)  # 0 means default camera

# Check if the camera is opened successfully
if not capture.isOpened():
    print("Error: Couldn't open the camera.")
    exit()

while rclpy.ok():
    # Capture a frame
    ret, frame = capture.read()

    # Check if the frame was captured successfully
    if not ret:
        print("Error: Couldn't capture frame.")
        break

    # Perform inference on the captured frame
    start = time.time()
    output_data = inference(frame)
    # print("Inference Time:", time.time() - start)
    # print("Output:", output_data)

    # Create a Twist message with the output data
    twist_msg = Twist()
    twist_msg.linear.x = float(output_data[0][0])
    twist_msg.angular.z = float(output_data[0][1])

    # Publish the Twist message
    publisher.publish(twist_msg)

    # Check for 'q' key to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera capture
capture.release()
cv2.destroyAllWindows()
node.destroy_node()
rclpy.shutdown()
