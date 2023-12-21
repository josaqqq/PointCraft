#!/bin/bash

echo "Please enter user id."
read userID

# for demonstration
../bin/PointCraft ./point_cloud_practice.obj 0 --spray --sketch

if ! [[ $userID =~ ^[0-9]+$ ]]; then
  echo "Please input integer."
elif [ $userID -le 4 ]; then
  # if userID <= 4, the user belongs to group A
  ../bin/PointCraft ./point_cloud_practice.obj $userID --spray
  ../bin/PointCraft ./point_cloud_practice.obj $userID --sketch

  ../bin/PointCraft ./point_cloud_1.obj $userID --spray
  ../bin/PointCraft ./point_cloud_1.obj $userID --sketch

  ../bin/PointCraft ./point_cloud_2.obj $userID --spray
  ../bin/PointCraft ./point_cloud_2.obj $userID --sketch
else
  # if userID >= 5, the user belongs to group B
  ../bin/PointCraft ./point_cloud_practice.obj $userID --sketch
  ../bin/PointCraft ./point_cloud_practice.obj $userID --spray

  ../bin/PointCraft ./point_cloud_1.obj $userID --sketch
  ../bin/PointCraft ./point_cloud_1.obj $userID --spray

  ../bin/PointCraft ./point_cloud_2.obj $userID --sketch
  ../bin/PointCraft ./point_cloud_2.obj $userID --spray
fi