/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Quick way of implementing Xbox joysticks and triggers using the {@link XboxController} class and
 * making them into {@link DoubleSupplier}s, so that they can be declared as constants and still 
 * return their respective axis values.
 */
public class ControllerAxis implements DoubleSupplier{

    private final XboxController controller;
    private final int AXIS;

    /**
     * Constructor for the specified controller axis. This takes in the controller and the ID of the 
     * intended axis and makes it a {@link DoubleSupplier}.
     * 
     * @param xboxController - the {@link XboxController} that the axis exists on
     * @param axisID - the ID of the corresponding axis (can be found in DriverStation)
     */
    public ControllerAxis(XboxController xboxController, int axisID){
        controller = xboxController;
        AXIS = axisID;
    }
    
    @Override
    public double getAsDouble(){
        return controller.getRawAxis(AXIS);
    }
    
}
