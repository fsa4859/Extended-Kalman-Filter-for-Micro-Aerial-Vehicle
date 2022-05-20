# Extended-Kalman-Filter-for-Micro-Aerial-Vehicle
This projects implements Extended Kalman Filter for Micro Aerial Vehicle to estimate the position, orientation and velocity. Control inputs are provided from on board IMU and measurement update is obtained from VICON system. The model is being tested on three datasets preprepared for testing.

# Process Flow
1. Developed state space system for the dynamic model
2. Computed the Jacobian to handle the non linearity by approximating around the mean values.
3. Implement discete form of EKF to be implemented in Matlab

# Results
## Measurement Model (Position and Orientation from Vicon)
### Dataset 1
![image](https://user-images.githubusercontent.com/69100847/169523840-c309f731-f806-4a4f-8f5d-27e79ce31f33.png)


### Dataset 4
![image](https://user-images.githubusercontent.com/69100847/169523924-42bf40d4-80eb-4e5b-87bd-1eeea87d7823.png)


### Dataset 9
![image](https://user-images.githubusercontent.com/69100847/169523971-9ee11c27-a906-4dac-a75a-74ac71d39fb9.png)


## Measurement Model (Velocity from Vicon)
### Dataset 1
![image](https://user-images.githubusercontent.com/69100847/169524209-e7141360-df04-42d7-8a5c-08d1ce1036da.png)


### Dataset 4
![image](https://user-images.githubusercontent.com/69100847/169524267-bd6bd606-4cb1-4391-8cd4-aac393714bcc.png)


### Dataset 9
![image](https://user-images.githubusercontent.com/69100847/169524333-5d53b727-4e20-4cb2-a373-5dc61fde586b.png)


