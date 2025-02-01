// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

/** 
 * an enum that represents the different sides of the reef. They are named as if they were viewed from the alliance wall, with front 
 * being the sides closest to the alliance wall, and back being the sides further from the alliance wall.
 * none is for when you are not within the threshold (as declared in drivetrainsubsystem) that marks you as close enough to a reef side to begin lineup*/
public enum ReefSideEnum {
    bargeFar,
    middleFar,
    processorFar,
    processorClose,
    middleClose,
    bargeClose,
    none
}
