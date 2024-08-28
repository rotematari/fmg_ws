#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import torch
import yaml
import logging
from sklearn.preprocessing import StandardScaler
import numpy as np 
import pandas as pd
from models.models import TransformerModel
from scipy.signal import convolve2d
import serial
import matplotlib.pyplot as plt
import os
from scipy.signal import savgol_filter
import time
from custom_msg.msg import ArmPositions
from ament_index_python.packages import get_package_share_directory
# Change the current working directory to the directory of the script
os.chdir(os.path.dirname(os.path.abspath(__file__)))



class RealTimeSystem(Node):
    
    """
    A class to handle real-time data processing, prediction, and visualization using a neural network model.
    """

    def __init__(self):
        super().__init__('realtime_prediction_pub_node')
        """
        Initialize the RealTimeSystem with a configuration file.

        Args:
            config_path (str): Path to the configuration YAML file.
        """
        self.get_logger().info('RealTimeSystem starting...')
        self.load_config()

        
        
        self.device = self.initialize_device()

        self.load_checkpoint(self.RealTimeconfig['checkpoint_path'])
        self.config = self.checkpoint['config']
        self.model = self.initialize_model()
        self.model.load_state_dict(self.checkpoint['model_state_dict'])
        self.feature_scaler, self.label_scaler = self.initialize_scaler()

        self.first = True
        
        if self.RealTimeconfig['testFromFile']:
            self.load_test_data()
        else:
            self.ser = self.initialize_serial()
            
        # Create a timer for periodic execution
        self.create_timer(0.015, self.run)
        self.pose_publisher = self.create_publisher(ArmPositions, 'prediction_pose', 10)
        
    def load_config(self,):
        self.RealTimeconfig = {}
        self.declare_parameters(
            namespace='',
            parameters=[
                ('device', 'cpu'),
                ('serial_port', '/dev/ttyACM0'),
                ('serial_baudrate', 115200),
                ('data_path', ""),
                ('input_featurs',['']),
                ('input_labels', ['']),
                ('sequence_length', 100),
                ('window_size', 5),
                ('calibration_length', 1000),
                ('testFromFile', False),
                ('maxpastsize', 100),
                ('checkpoint_path', '/home/ahmed/Downloads/TransformerModel.pth'),
            ])
        # Get all parameters
        all_params = self.get_parameters_by_prefix('')
        self.get_logger().info(f'Loaded parameters: {all_params}')
        # Iterate through all parameters and store them in the config dictionary
        for name, param in all_params.items():
            self.RealTimeconfig[name] = param.value
        
        # Now you can access parameters like this:
        # self.config['param1']
        # self.config['param2']
        
        # Print the config to verify
        self.get_logger().info(f'Loaded config: {self.RealTimeconfig}')

    def initialize_device(self):
        """Initialize the computation device (CPU or GPU)."""
        if self.RealTimeconfig["device"]== "cuda" and torch.cuda.is_available():
            self.get_logger().info(f"CUDA is available. Using GPU: {torch.cuda.get_device_name(0)}")
            return torch.device("cuda:0")
        else:
            self.get_logger().info("CUDA is not available or CPU is selected. Using CPU.")
            return torch.device("cpu")

    def initialize_model(self):
        """
        Initialize and return the model.

        Returns:
            model: The initialized model.
        """
        return TransformerModel(self.config).to(self.device)
    
    def initialize_serial(self):
        """
        Initialize and return the serial connection.

        Returns:
            Serial: The initialized serial connection.
        """
        ser = serial.Serial(self.RealTimeconfig['serial_port'], self.RealTimeconfig['serial_baudrate'])
        for _ in range(100):
            ser.readline()
        return ser 
    
    def initialize_scaler(self):
        """
        Initialize and return the feature and label scalers.

        Returns:
            tuple: A tuple containing the feature and label scalers.
        """
        label_scaler = StandardScaler()
        feature_scaler = StandardScaler()
        std_feature_scaler_state = self.checkpoint['std_feature_scaler_state']
        std_label_scaler_state = self.checkpoint['std_label_scaler_state']
        
        label_scaler.mean_ = np.array(std_label_scaler_state['mean'])
        label_scaler.var_ = np.array(std_label_scaler_state['var'])
        label_scaler.scale_ = np.array(std_label_scaler_state['scale'])
        label_scaler.n_samples_seen_ = std_label_scaler_state['n_samples_seen']
        
        feature_scaler.mean_ = np.array(std_feature_scaler_state['mean'])
        feature_scaler.var_ = np.array(std_feature_scaler_state['var'])
        feature_scaler.scale_ = np.array(std_feature_scaler_state['scale'])
        feature_scaler.n_samples_seen_ = std_feature_scaler_state['n_samples_seen']
        
        return feature_scaler, label_scaler

    def load_checkpoint(self, checkpoint_path):
        """
        Load the model checkpoint.

        Args:
            checkpoint_path (str): Path to the model checkpoint file.
        """
        # Get the path to the shared directory of the specified package
        shared_dir = get_package_share_directory("RealTimeFmgPrediction_pkg")
        
        # Construct the full path to the checkpoint file
        checkpoint_path = os.path.join(shared_dir, checkpoint_path)

        self.checkpoint = torch.load(checkpoint_path)

    def load_test_data(self):
        """Load test data from files."""
        self.data = pd.read_csv(self.RealTimeconfig['data_path'])
        self.testInputs = self.data[self.RealTimeconfig['input_featurs']].to_numpy()
        self.testInputsLabes = self.data[self.RealTimeconfig['input_labels']].to_numpy()
        self.testInputs = self.feature_scaler.transform(self.testInputs)
        self.testInputs_seq = self.create_sliding_sequences(self.testInputs, self.config.sequence_length)
        self.testInputs_labels_seq = self.create_sliding_sequences(self.testInputsLabes, self.config.sequence_length)
        self.testIndex = 0

    def calibrate_system_if_needed(self):
        """Calibrate the system if needed."""
        calibration_length = self.RealTimeconfig['calibration_length']
        self.get_logger().info("Start calibration\n----------------\n")
        self.calibrate_system(calibration_length=calibration_length)
        self.get_logger().info("End calibration\n----------------\n")

    def calibrate_system(self, calibration_length):
        """
        Read calibration data and set up bias and scale parameters.

        Args:
            calibration_length (int): Number of data points for calibration.
        """
        Caldata = []

        for i in range(calibration_length):
            Caldata.append(self.readline())
            if i % 100 == 0:
                self.get_logger().info(f'{i} points collected')

        Caldata = np.array(Caldata, dtype=np.float32)
        self.session_bias = Caldata.mean(axis=0)

    def create_sliding_sequences(self, input_array: np.ndarray, sequence_length: int):
        """
        Create sliding sequences from the input array.

        Args:
            input_array (numpy.ndarray): Input data array.
            sequence_length (int): Length of each sequence.

        Returns:
            numpy.ndarray: Array of sequences.
        """
        sample_size, features = input_array.shape
        new_sample_size = sample_size - sequence_length + 1

        sequences = [input_array[i:i+sequence_length] for i in range(new_sample_size)]
        return np.array(sequences)
    
    def readline(self):
        """
        Read a line of data from the serial connection.

        Returns:
            list: A list of data points.
        """
        try:
            self.ser.reset_input_buffer()
            
            line = self.ser.readline().decode("utf-8").rstrip(',\r\n').split(',')
            line = self.ser.readline().decode("utf-8").rstrip(',\r\n').split(',')
            
            if len(line) == 32:
                DataPoint = [int(num) for num in line]
                print(DataPoint[-1])
                # self.last_Data_Point = DataPoint
                return DataPoint[:32]
            else:
                self.get_logger().warning(f"Bad reading: Expected 32 data points, got {len(line)}")
                return None
        except Exception as e:
            self.get_logger().error(f'Error reading line: {e}')
            return None

    def read_seq(self):
        """
        Read a sequence of data points.

        Returns:
            numpy.ndarray: Processed sequence of data points.
        """
        window_size = self.config.window_size
        sequence_length = self.config.sequence_length + window_size - 1
        
        if not self.RealTimeconfig['testFromFile']:
            sequence = self.initialize_sequence(sequence_length)
            sequence = self.process_sequence(sequence, window_size)
        else:
            self.testInputs
            sequence = self.testInputs_seq[self.testIndex]
            self.testIndex += 1

        return sequence

    def get_good_reading(self):
        """
        Keep trying to get a good reading.

        Returns:
            list: A valid data point.
        """
        while True:
            line = self.readline()
            if line:
                return line
    
    def initialize_sequence(self, sequence_length):
        """
        Initialize the sequence for real-time data.

        Args:
            sequence_length (int): The length of the sequence.

        Returns:
            list: Initialized sequence.
        """
        sequence = []
        if self.first:
            while len(sequence) < sequence_length:
                sequence.append(self.get_good_reading())
            self.last_sequence = sequence
            self.first = False
        else:
            new_reading = self.get_good_reading()
            self.last_sequence = self.last_sequence[1:]
            self.last_sequence.append(new_reading)
            
        return self.last_sequence

    def process_sequence(self, sequence, window_size):
        """
        Process the sequence by applying calibration, moving average, and scaling.

        Args:
            sequence (list): The sequence to process.
            window_size (int): The window size for moving average.

        Returns:
            numpy.ndarray: Processed sequence.
        """
        try:
            # sequence = np.array(sequence)
            # sliding_window = np.lib.stride_tricks.sliding_window_view(sequence, window_size, axis=0)
            # sequence = sliding_window.mean(axis=2)
            # TODO write a func to apply filter 
            
            sequence = self.feature_scaler.transform(sequence)
        except Exception as e:
            self.get_logger().error('Error in process sequence : %s', e)
            sequence = None
        return sequence

    def predict(self, sequence):
        """
        Predict using the model.

        Args:
            sequence (numpy.ndarray): Input sequence.

        Returns:
            numpy.ndarray: Model prediction.
        """
        self.model.eval()
        with torch.no_grad():

            sequence = torch.tensor(sequence, dtype=torch.float32, device=self.device).unsqueeze(0)
            try:
                prediction = self.model(sequence)
                prediction = prediction[:, -1, :]
                prediction = self.label_scaler.inverse_transform(prediction.detach().cpu().numpy())
            except Exception as e:
                self.get_logger().error(f'Error in prediction: {e}')
                prediction = None

        return prediction
    def numpy_ewma_vectorized_v2(self,data, window):
        alpha = 2 / (window + 1.0)
        alpha_rev = 1 - alpha
        n = data.shape[0]

        pows = alpha_rev**(np.arange(n + 1))

        scale_arr = 1 / pows[:-1]
        offset = data[0] * pows[1:]
        pw0 = alpha * alpha_rev**(n - 1)

        mult = data * pw0 * scale_arr
        cumsums = mult.cumsum()
        out = offset + cumsums * scale_arr[::-1]
        return out

    def apply_numpy_ewma(self,data, window):
        batch, sequence, features = data.shape
        result = np.zeros_like(data)
        for b in range(batch):
            for f in range(features):
                result[b, :, f] = self.numpy_ewma_vectorized_v2(data[b, :, f], window)
        return result
    
    def apply_savgol_filter(self,prediction: np.ndarray)-> np.ndarray:
        
        if hasattr(self, 'pastPredictions'):
            if self.pastPredictions.shape[0] < self.RealTimeconfig["maxpastsize"]:
                self.pastPredictions = np.vstack((self.pastPredictions,prediction))
            else:
                self.pastPredictions = np.vstack((self.pastPredictions[1:],prediction))
        else:
            self.pastPredictions = prediction
        
        if self.pastPredictions.shape[0] > 5:
            #calc slagov filter loc Vel
            if self.pastPredictions.shape[0]<100:
                window_size = self.pastPredictions.shape[0]
            else:
                window_size = 100
            
            # poly_order = 5
            # pastPredictions = savgol_filter(self.pastPredictions,deriv=0, window_length=window_size, polyorder=poly_order,axis=0)
            # firstOrder_derivative_predsToPlot = savgol_filter(self.pastPredictions,deriv=1,delta=30.0, window_length=window_size, polyorder=poly_order,axis=0)
            # # calc new loc 
            # prediction = pastPredictions[-1] 
            # prediction += firstOrder_derivative_predsToPlot[-1]*100

        return prediction
    def calculate_quaternion(self,point_1, point_2):
        
        # Step 1: Calculate the vector from the point_1 to the point_2
        vector = np.array(point_2) - np.array(point_1)
        position = vector/2
        # Step 2: Normalize the vector
        vector_norm = vector / np.linalg.norm(vector)
        
        # Step 3: Initial z-axis (0, 0, 1)
        z_axis = np.array([0, 0, 1])

        # Step 4: Calculate the rotation axis (cross product of z_axis and vector_norm)
        rotation_axis = np.cross(z_axis, vector_norm)
        
        # Step 5: Calculate the angle (dot product of z_axis and vector_norm)
        angle = np.arccos(np.dot(z_axis, vector_norm))
        
        # Step 6: Normalize the rotation axis
        if np.linalg.norm(rotation_axis) != 0:
            rotation_axis_norm = rotation_axis / np.linalg.norm(rotation_axis)
        else:
            rotation_axis_norm = np.array([0, 0, 0])
        
        # Step 7: Calculate the quaternion
        w = np.cos(angle / 2)
        x, y, z = rotation_axis_norm * np.sin(angle / 2)
        
        quaternion = np.array([x, y, z, w])
        
        # return rotation.as_quat()
        return quaternion , position
    
    def publish_pose(self, prediction):
        
        prediction = prediction/100
        shoulder_pred = np.array(prediction[:3], dtype=float)
        elbow_pred = np.array(prediction[3:6], dtype=float)
        wrist_pred = np.array(prediction[6:9], dtype=float)

        # self.get_logger().info(f"Shoulder: {shoulder_pred}, Elbow: {elbow_pred}, Wrist: {wrist_pred}")
        # Create an ArmPositions message
        arm_positions_msg = ArmPositions()
        
        QsholderToElbow,shoulder_position = self.calculate_quaternion(shoulder_pred, elbow_pred)
        QelbowToWrist, elbow_position = self.calculate_quaternion(elbow_pred, wrist_pred)
        # self.get_logger().info(f"Shoulder Position: {shoulder_pred[0] + shoulder_position[0]}, Elbow Position: {elbow_position}")

        arm_positions_msg.upper_arm.position.x = shoulder_pred[0] + shoulder_position[0]
        arm_positions_msg.upper_arm.position.y = shoulder_pred[1] + shoulder_position[1]
        arm_positions_msg.upper_arm.position.z = shoulder_pred[2] + shoulder_position[2]
        
        arm_positions_msg.upper_arm.orientation.x = QsholderToElbow[0]
        arm_positions_msg.upper_arm.orientation.y = QsholderToElbow[1]
        arm_positions_msg.upper_arm.orientation.z = QsholderToElbow[2]
        arm_positions_msg.upper_arm.orientation.w = QsholderToElbow[3]

        arm_positions_msg.lower_arm.position.x = elbow_pred[0] + elbow_position[0]
        arm_positions_msg.lower_arm.position.y = elbow_pred[1] +elbow_position[1]
        arm_positions_msg.lower_arm.position.z = elbow_pred[2] +elbow_position[2]
        arm_positions_msg.lower_arm.orientation.x = QelbowToWrist[0]
        arm_positions_msg.lower_arm.orientation.y = QelbowToWrist[1]
        arm_positions_msg.lower_arm.orientation.z = QelbowToWrist[2]
        arm_positions_msg.lower_arm.orientation.w = QelbowToWrist[3]

        # Publish the Pose message
        self.pose_publisher.publish(arm_positions_msg)
    
    def run(self):
        """Main loop for real-time data handling."""
        sequence = self.read_seq()
        if sequence is not None and len(sequence) > 0:
            prediction = self.predict(sequence)
            if prediction is not None and len(prediction) > 0:
                prediction = self.apply_savgol_filter(prediction)


                self.publish_pose(prediction[0])
            else:
                self.get_logger().warning("prediction was None")
        else:
            self.get_logger().warning("sequence was None")

    def get_ground_truth(self):
        """
        Get ground truth data.

        Returns:
            dict: Ground truth data.
        """
        if self.RealTimeconfig['testFromFile']:
            ground_truth = {
                'chest': [(0, 0, 0)],
                'shoulder': [self.testInputs_labels_seq[self.testIndex, -1, 0:3]],
                'elbow': [self.testInputs_labels_seq[self.testIndex, -1, 3:6]],
                'wrist': [self.testInputs_labels_seq[self.testIndex, -1, 6:9]],
                'table_base': [(0, 0, 0)],
            }
        else:
            try:
                ground_truth = self.natnet_reader.read_sample()
                if  len(ground_truth['shoulder']) == 0:
                    ground_truth = {
                    'chest': [(0, 0, 0)],
                    'shoulder': [(0, 0, 0)],
                    'elbow': [(0, 0, 0)],
                    'wrist': [(0, 0, 0)],
                    'table_base': [(0, 0, 0)],
                }

            except Exception as e:
                self.get_logger().error(f'Error in NatNetReader: {e}')
                ground_truth = {
                    'chest': [(0, 0, 0)],
                    'shoulder': [(0, 0, 0)],
                    'elbow': [(0, 0, 0)],
                    'wrist': [(0, 0, 0)],
                    'table_base': [(0, 0, 0)],
                }
        return ground_truth




def main():
    config_path = 'config/RealTimeConfig.yaml'
    rclpy.init()
    realtime_node = RealTimeSystem()

    try:
        rclpy.spin(realtime_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        realtime_node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()


