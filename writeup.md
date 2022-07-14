# Writeup: 3D Object detection

## Vehicle characteristics

The following features appear recurrently on cars in the LIDAR data:

Front/rear view:
- Lights: Are clearly visbile in the intensity channel but not in spatial data
- License plates: Also visible in the intensity channel
- Glass: Since light can go through it, points do not appear on glass surfaces making "holes" on the car.
- Bumpers: Appear in the spatial data always as volumetric features with similar sizes

Side view:
- Tires: Appear in the intensity channel

The following pointclouds and intensity images show this characteristics. Is important to keep in mind that occlusion often covers part of the cars, as shown in the images as well.

![Screenshot from 2022-06-26 18-50-00](https://user-images.githubusercontent.com/71234974/175839520-66cd5121-6b9b-4bab-bb8a-6a4e8c655ba8.png)
Lights and glasses appear in the pointcloud and range image
![Screenshot from 2022-06-26 18-53-53](https://user-images.githubusercontent.com/71234974/175839526-595bdcad-31ed-4724-9dfd-4c79c9c2c237.png)
Tires appear on cars when looked on the sides
![Screenshot from 2022-06-26 18-54-28](https://user-images.githubusercontent.com/71234974/175839534-e28eea6f-7f9f-48a8-a536-9203d6bfae49.png)
Cars close to the LIDAR are partly occluede because remain outside of the FoV
![Screenshot from 2022-06-26 18-55-35](https://user-images.githubusercontent.com/71234974/175839539-b8d90632-07f9-4ca0-9f8c-8b6bc8517be2.png)
Car chassis
![Screenshot from 2022-06-26 18-57-20](https://user-images.githubusercontent.com/71234974/175839606-28e9dca0-981d-40eb-8f69-60aab1a81045.png)
![Screenshot from 2022-06-26 18-58-06](https://user-images.githubusercontent.com/71234974/175839615-bc9070f7-f6e2-4f86-a2d8-d06267e68140.png)
![Screenshot from 2022-06-26 18-58-35](https://user-images.githubusercontent.com/71234974/175839618-ad1aaba4-9998-4993-98a6-e247d53ed13c.png)
Cars can occlude other cars when they fall in front of them
![Screenshot from 2022-06-26 18-59-42](https://user-images.githubusercontent.com/71234974/176007351-37496c76-0647-4c22-a075-1f621f49630e.png)
Trucks can cause big shadows over other cars, and their general dimensions are very different from other cars
![Screenshot from 2022-06-26 19-00-54](https://user-images.githubusercontent.com/71234974/175839629-118cc932-c94b-4c7e-8588-d1f6572c20d9.png)
![Screenshot from 2022-06-26 19-00-14](https://user-images.githubusercontent.com/71234974/176007114-4a900d9b-c41a-4885-bb0f-15209bfdcb02.png)

Less points appear on cars as they fall further away from the sensor

# BeV extraction
The pictures below show the BeV image of the LIDAR pointcloud for the intensity and height channels in that order

![Screenshot from 2022-06-28 21-12-45](https://user-images.githubusercontent.com/71234974/176336437-2b691226-3d34-4dac-bae5-e2da021bb183.png)

![Screenshot from 2022-06-28 21-13-02](https://user-images.githubusercontent.com/71234974/176336434-0936747a-dba6-41b9-b194-7355fc585a37.png)

# Object detection and projections
In the image below you can see the cars detected on the BeV view, as well as the 3D bounding boxes projected over the RGB camera image

![Screenshot from 2022-06-28 21-09-45](https://user-images.githubusercontent.com/71234974/176335934-2260c991-a083-49b3-8b58-2458cd0e1b50.png)


# Performance evaluatuion

The plots below show the distribution of performance metrics on detections performed over 100 samples in the dataset. Results may vary due to the IoU threshold selected
![Screenshot from 2022-06-28 20-57-12](https://user-images.githubusercontent.com/71234974/176334633-3374a313-cad6-41eb-a7cc-d4d61fbbb247.png)

When labels are used as detections, perfect scores are achieved as shown below: 
![Screenshot from 2022-06-28 21-04-12](https://user-images.githubusercontent.com/71234974/176335633-cd3483ee-e09d-4d78-a4fe-4a36c3632045.png)


# Writeup: Track 3D-Objects Over Time

RMSE error of single target tracking:

![rmes1](https://user-images.githubusercontent.com/71234974/178859162-5e7eec21-0bb5-4659-af22-e37445ea993e.png)


RMSE with track mangement 

![ex2](https://user-images.githubusercontent.com/71234974/178127195-6a33cb30-c9f5-4caa-8d55-59265fbfdced.png)

Multi object RMSE

![Figure_2](https://user-images.githubusercontent.com/71234974/178128614-22c3841e-6f1e-4cf2-8fd4-5de67c32f2e8.png)

Tracking video was uploaded to google drive to avoid pushing heavy files to github. You can see it in this [link](https://drive.google.com/file/d/1SoQVw_RS9_CNYwROMHSt8Cv6KqURFsdf/view?usp=sharing). The final RMSE plot is shown below:

![rmsefinal](https://user-images.githubusercontent.com/71234974/178858440-eb4ad59e-dad6-4774-b692-48105704b1c0.png)


Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

The implementation had the following steps:

1. Filter: A kalman filter class was implemented for tracking 6D movement of objects. 

2. A tracking module was implemented to store the state of each tracked object over time and associate metadata like the score of the tracking result, the state of the track, etc.

3. An association module was implemented to assign measurements to each of the tracked tracks according to the Mahalanobis distance.

4. A camera sensor object was added to calculate appropriate non-linear jacobians to feed to the Kalman Filter created in the first step.

The hardest part of the project was integrating the camera model, since the rest of the classes had to be compatible with two types of measurements with different dimensions: pixels (2D) coming from the camera and 3D lidar mesurements

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 

When the camera is integrated, RMSE is reduced. Additionally, ghost tracks occur less frequently and disappear faster when created by ghost/spurious detections on LIDAR data. In theory is also beneficial because it adds an extra source of information to the system, which should in general decrease the uncertanty (gausssian variance) of the final measured states.

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?

Occlusion of objects, weather condition that hurt the performance of specific sensors like cameras, limited field of view, limited processing capacity for taking measurements from several data sources in real time.

From the above, mainly occlusion was present on the project


### 4. Can you think of ways to improve your tracking results in the future?

Tuning the noise parameters of the sensor may improve the model. Also, the association module could use a more sophisticated technique. Other object detection models may be tried out or trained.
