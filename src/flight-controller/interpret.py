import logging

import pandas as pd
import requests

import matplotlib.pyplot as plt

logger=logging.getLogger(__name__)

def download_csv_data(url, file_path):
    logger.info(f'Downloading data from {url} to {file_path}')
    try:
        response = requests.get(url + "/reset", timeout=0.1)
        response.raise_for_status()
    except requests.exceptions.RequestException as e:
        logger.info(f'Failed to reset data: {e}')
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
    plt.plot(data['time'], data['raw_roll'], label='Raw Roll')
    plt.plot(data['time'], data['raw_pitch'], label='Raw Pitch')
    plt.plot(data['time'], data['raw_yaw'], label='Raw Yaw')
    plt.xlabel('Time')
    plt.ylabel('Raw Values')
    plt.legend()
    
    # Plot errors and PID values
    plt.subplot(3, 1, 3)
    plt.plot(data['time'], data['motor_one'], label='Motor 1')
    plt.plot(data['time'], data['motor_two'], label='Motor 2')
    plt.plot(data['time'], data['motor_three'], label='Motor 3')
    plt.plot(data['time'], data['motor_four'], label='Motor 4')
    plt.xlabel('Time')
    plt.ylabel('Motor Values')
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
