// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class Button {
    public static XboxController controller1 = new XboxController(0);

    public static Trigger leftTriggerDriver =  new Trigger(() -> controller1.getRawAxis(2) > 0.1);
    public static Trigger rightTriggerDriver = new Trigger(() -> controller1.getRawAxis(3) > 0.1);
}
