/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;
import frc.robot.commands.CmdDrive;
import frc.robot.commands.CmdDriveConnor;

/**
 * Add your docs here.
 */
public class Base extends Subsystem {
  private MecanumDrive mecDrive;
  private CANSparkMax sparkFR, sparkFL, sparkBL, sparkBR;
  private Solenoid solToggle;
  private boolean useMecanum;
  private SpeedControllerGroup leftGroup, rightGroup;
  private DifferentialDrive diffDrive;

  public Base() {
    sparkFR = new CANSparkMax(RobotMap.MOTOR_FR, MotorType.kBrushless);
    sparkFL = new CANSparkMax(RobotMap.MOTOR_FL, MotorType.kBrushless);
    sparkBR = new CANSparkMax(RobotMap.MOTOR_BR, MotorType.kBrushless);
    sparkBL = new CANSparkMax(RobotMap.MOTOR_BL, MotorType.kBrushless);
    leftGroup = new SpeedControllerGroup(sparkFL, sparkBL);
    rightGroup = new SpeedControllerGroup(sparkFR, sparkBR);
    solToggle = new Solenoid(14, RobotMap.TOGGLE);

    mecDrive = new MecanumDrive(sparkFL, sparkBL, sparkFR, sparkBR);
    diffDrive = new DifferentialDrive(leftGroup, rightGroup);
  }

  public void extendToggle(boolean x) {
    solToggle.set(useMecanum);
  }

   public boolean usingMecanum() {
    return useMecanum;
  }

  public void toggleMecanum() {
    enableMecanum(!usingMecanum());
  }

  public void enableMecanum(boolean x) {
    useMecanum = x;
    extendToggle(useMecanum);
  }

  public void drive(float leftX, float leftY, float rightX) {
    mecDrive.driveCartesian(useMecanum?leftX:0, -leftY, rightX);
  }

  public void tankDrive(float left, float right) {
    diffDrive.tankDrive(left, right);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CmdDriveConnor());
  }
}
