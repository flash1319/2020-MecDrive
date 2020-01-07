/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.CmdToggleControls;
import frc.robot.commands.CmdToggleMode;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public XboxController driver;

  public JoystickButton driverA;
  public JoystickButton driverX;
  public JoystickButton driverLB;
  public JoystickButton driverRB;
  public JoystickButton driverStart;

  public OI() {
    driver = new XboxController(0);
    
    driverA = new JoystickButton(driver, XboxMap.XB360_A);
    driverLB = new JoystickButton(driver, XboxMap.XB360_BUMPER_LEFT);
    driverRB = new JoystickButton(driver, XboxMap.XB360_BUMPER_RIGHT);
    driverStart = new JoystickButton(driver, XboxMap.XB360_START);

    driverX = new JoystickButton(driver, XboxMap.XB360_B);
    driverX.whenPressed(new CmdToggleMode());
    driverStart.whenPressed(new CmdToggleControls());
  }

  public double getDriverRightT() {
    return Util.clampAbs(0.1, driver.getTriggerAxis(Hand.kRight));
  }

  public double getDriverLeftT() {
    return Util.clampAbs(0.1, driver.getTriggerAxis(Hand.kLeft));
  }

  public double getDriverRightY() {
    return Util.clampAbs(0.05, driver.getY(Hand.kRight)-0.02);
  }

  public double getDriverRightX() {
    return Util.clampAbs(0.05, driver.getX(Hand.kRight) + 0.03);
  }

  public double getDriverLeftY() {
    return Util.clampAbs(0.05, driver.getY(Hand.kLeft));
  }

  public double getDriverLeftX() {
    return Util.clampAbs(0.05, driver.getX(Hand.kLeft) - 0.04);
  }
}
