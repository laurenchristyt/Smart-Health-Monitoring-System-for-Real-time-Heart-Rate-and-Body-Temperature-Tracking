# Smart Health Monitoring System for Real-Time Heart Rate and Body Temperature Tracking

## Introduction to the problem and the solution

In the ever-evolving landscape of healthcare, there is a growing need for innovative and efficient solutions that empower individuals to monitor their health in real-time. Recognizing this demand, we present the "Smart Health Monitoring System for Real-Time Heart Rate and Body Temperature Tracking," an IoT project aimed at seamlessly integrating technology into personal health management.

Traditional healthcare monitoring often involves periodic visits to medical facilities, limiting the continuous tracking of vital health parameters. This approach not only poses challenges in terms of accessibility but also may lead to delayed detection of critical health issues. In light of these constraints, there is a pressing need for a solution that enables individuals to monitor their heart rate and body temperature in real-time, providing timely insights and facilitating proactive healthcare management.

Our project leverages cutting-edge technology to address these challenges. The system incorporates the ESP32 microcontroller, MAX30100 pulse oximeter sensor, and LM35 temperature sensor, forming a robust framework for real-time health data acquisition. The ESP32 acts as the central processing unit, orchestrating the seamless integration of sensor data into a cohesive and informative health monitoring system.

The MAX30100 sensor, renowned for its accuracy and efficiency, is employed to monitor heart rate and oxygen saturation levels. Simultaneously, the LM35 sensor captures body temperature data with high precision. This amalgamation of sensors ensures comprehensive health monitoring, enabling users to gain insights into their cardiovascular health and overall well-being.

To enhance user accessibility and convenience, our solution integrates with Thingsboard, a powerful IoT platform. Thingsboard serves as the bridge between the hardware components and the user interface, facilitating the seamless transmission of health data. The platform allows users to visualize their heart rate and body temperature trends over time, empowering them to make informed decisions about their health.

Additionally, the user-friendly interface extends to smartphones, providing a convenient and portable means of accessing health data. Through a dedicated mobile application, users can monitor their vital signs in real-time, set personalized health goals, and receive alerts for abnormal readings. This integration with smartphones ensures that health monitoring is not confined to a specific location, enabling users to stay informed and proactive about their well-being wherever they go.

## Hardware design and implementation details

The Smart Health Monitoring System for Real-time Heart Rate and Body Temperature Tracking is an intelligent monitoring system capable of tracking users' heart rate and body temperature in real-time. The project aims to provide accurate and instant vital information for personal health monitoring or medical supervision. To implement the design of this project, several hardware components are required, including the ESP32, MAX30100 sensor, and LM35 sensor.

The ESP32 is used as the central component, serving as the main brain of the system. The ESP32's capabilities in managing WiFi and Bluetooth connectivity are crucial for sending data to the cloud platform or mobile application to display information to users through an interactive user interface.

The MAX30100 is a versatile sensor specifically designed for heart rate measurement and blood oxygen saturation (SpO2) applications. This oximeter sensor operates on the I2C bus, connecting the I2C pins (SCL & SDA) from the oximeter module to GPIO 21 and GPIO 22 on the ESP32. Additionally, the INT pin is connected to GPIO 19 on the ESP32 as an analog input/output pin to transmit data.

The LM35 is an analog temperature sensor designed to measure the user's body temperature in degrees Celsius. The LM35 sensor has three pins: the VCC pin connected to Vin, the GND pin connected to ground, and the OUT pin connected to GPIO 36 or ADC0 for analog data transmission.

Readings from the MAX30100 sensor (heart rate) and LM35 sensor (body temperature) can be displayed in real-time through the Serial Monitor in the Arduino IDE and sent over the internet to a mobile application using the Thingsboard platform. This is intended to make it easy for users to access their readings through their smartphones.

## Software implementation details

## Test results and performance evaluation

## Conclusion and future work

THe Smart Health Monitoring System successfully integrates the ESP32, MAX30100 sensor, and LM35 sensor to provide real-time tracking of heart rate and body temperature. The ESP32 serves as the central hub, utilizing WiFi and Bluetooth for seamless data transmission to a cloud platform or mobile application, enabling users to conveniently access vital health information. The MAX30100 sensor proves versatile for heart rate and blood oxygen saturation measurement, enhancing overall system functionality. Similarly, the LM35 sensor accurately measures body temperature, contributing valuable health data. Real-time readings are accessible through the Arduino IDE Serial Monitor, while integration with the Thingsboard platform facilitates remote monitoring via a mobile application. Looking ahead, potential future enhancements include improved data visualization, an alert system for abnormal readings, integration with wearable devices, implementation of machine learning algorithms for personalized insights, and the expansion of sensor capabilities to monitor additional health parameters, collectively evolving the Smart Health Monitoring System into a comprehensive solution for personal health management and medical supervision.