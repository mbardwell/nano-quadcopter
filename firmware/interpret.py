import logging

import pandas as pd
import requests

import matplotlib.pyplot as plt

logger=logging.getLogger(__name__)

def download_csv_data(url, file_path):
    logger.info(f'Downloading data from {url} to {file_path}')
    response = requests.get(url + "/reset", timeout=1)
    response.raise_for_status()
    response = requests.get(url + "/download", timeout=3)
    response.raise_for_status()
    with open(file_path, 'wb') as file:
        file.write(response.content)

# Function to read CSV and plot data
def plot_csv_data(file_path):
    logger.info(f'Plotting data from {file_path}')
    # Read the CSV file
    data = pd.read_csv(file_path)
    
    # Plot each column
    plt.figure(figsize=(15, 10))
    
    # Plot time vs throttle
    plt.subplot(3, 1, 1)
    plt.plot(data['time'], data['throttle'], label='Throttle')
    plt.xlabel('Time')
    plt.ylabel('Throttle')
    plt.legend()
    
    # Plot input and desired values
    plt.subplot(3, 1, 2)
    plt.plot(data['time'], data['input_roll'], label='Input Roll')
    plt.plot(data['time'], data['input_pitch'], label='Input Pitch')
    plt.plot(data['time'], data['input_yaw'], label='Input Yaw')
    plt.plot(data['time'], data['desired_roll'], label='Desired Roll')
    plt.plot(data['time'], data['desired_pitch'], label='Desired Pitch')
    plt.plot(data['time'], data['desired_yaw'], label='Desired Yaw')
    plt.xlabel('Time')
    plt.ylabel('Input/Desired Values')
    plt.legend()
    
    # Plot errors and PID values
    plt.subplot(3, 1, 3)
    plt.plot(data['time'], data['error_roll'], label='Error Roll')
    plt.plot(data['time'], data['error_pitch'], label='Error Pitch')
    plt.plot(data['time'], data['error_yaw'], label='Error Yaw')
    plt.plot(data['time'], data['pid_roll'], label='PID Roll')
    plt.plot(data['time'], data['pid_pitch'], label='PID Pitch')
    plt.plot(data['time'], data['pid_yaw'], label='PID Yaw')
    plt.xlabel('Time')
    plt.ylabel('Errors/PID Values')
    plt.legend()
    
    plt.tight_layout()
    plt.show()

def main():
    url = 'http://192.168.42.1:4242'
    file_path = 'downloaded_data.csv'
    download_csv_data(url, file_path)
    plot_csv_data(file_path)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
