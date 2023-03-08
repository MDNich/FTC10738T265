# About

This repository contains a code dump of code used in the FTC 2020-2021 Ultimate Goal season. 

Most of it is authored by myself, Marc D. Nichitiu, and my teammates Dennis Y and Michael M.

This code is open source and is to be cited when used. 

The highlights of this code are an interface between the intel T265 Tracking camera and RoadRunner, a path planning system for FTC Robots. This is adapted from pietroglyph's ftc265 project.

Thank you.

Marc D. Nichitiu, 10738 Bears, former Head Programmer.



# Citation

<code>This code was copied/modified from https://github.com/MDNich/FTC10738T265/, created by the FTC Team 10738 SBS Bears, specifically Marc D. Nichitiu, Dennis Y, and Michael M. There is no warranty whatsoever associated with this code. Please use responsibly and be sure to cite us in your code and notebook!</code>


# Navigation

The main T265 interfacing class is the org.sbs.bears.robot.controller.CamController class. Use this class to instantiate control with the camera object and to access position data. An example interface with RoadRunner can be found in the org.firstinspires.teamcode.drive.T265Localizer class, where the T265 IMU and Position system provides absolute positioning of the robot in a system relative to the starting psoition.

Please feel free to dm Marc on Discord at СтелелеМинунате#4075
