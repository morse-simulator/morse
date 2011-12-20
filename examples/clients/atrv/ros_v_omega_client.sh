#!/bin/sh

while :
do
  echo "Select an option:"
  echo "a) Enter speed"
  echo "b) Read coordinates"
  echo "q) Quit client program"
  read -p "Enter option: " OPTION
  case $OPTION in
  a)
    read -p "Enter V speed: " SPEEDV
    read -p "Enter W speed: " SPEEDW
    rostopic pub /ATRV/Motion_Controller geometry_msgs/Twist -1 [$SPEEDV,0,0] [0,0,$SPEEDW];;
  b)
    rostopic echo -n 1 /ATRV/Gyroscope;;
  q)
    break;;
  *)
    echo "Unknown option. Try again."
  esac
done
