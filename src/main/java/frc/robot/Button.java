// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class Button {
    public static PS4Controller controller1 = new PS4Controller(0);

    public static Trigger leftTriggerDriver = new JoystickButton(controller1, 2);
    public static Trigger rightTriggerDriver = new JoystickButton(controller1, 8);
}
