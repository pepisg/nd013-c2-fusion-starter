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


# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?


### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?


### 4. Can you think of ways to improve your tracking results in the future?

