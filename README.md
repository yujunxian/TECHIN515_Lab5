# TECHIN515 Lab 5 - Edge-Cloud Offloading

This lab builds on Lab 4's gesture classification system and introduces an edge-cloud offloading strategy. The ESP32 device performs local inference for gesture recognition, and only offloads raw sensor data to a cloud server when local confidence is low. This approach balances latency, accuracy, and connectivity needs.

## Completed Steps
- Created Azure resource group and ML workspace  
- Uploaded dataset to Azure Blob and trained gesture classifier  
- Registered model and deployed inference endpoint  
- Built local Flask web server to mimic cloud behavior  
- Implemented confidence-based offloading in ESP32 sketch  
- Documented both local and cloud inference with screenshots  
