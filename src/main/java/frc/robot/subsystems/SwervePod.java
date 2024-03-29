// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwervePod extends SubsystemBase {
  private CANSparkMax driveMotor;
  private CANSparkBase swerveMotor;
  private CANcoder dirEnc;
  private RelativeEncoder driveEnc;
  private int positionID;


  public SwervePod(int positionID, int podID, boolean emergencySparkMax) {
    this.positionID = positionID;
    driveMotor = new CANSparkMax(positionID, MotorType.kBrushless);
    if(emergencySparkMax){
      swerveMotor = new CANSparkMax(podID + 10, MotorType.kBrushless);
    }
    else{
      swerveMotor = new CANSparkFlex(podID + 10, MotorType.kBrushless);
    }
    

    dirEnc = new CANcoder(podID + 20);

    driveEnc = driveMotor.getEncoder();
    driveEnc.setPositionConversionFactor(((4/12.0) * Math.PI) / (8.14)); //circumference for 4" wheel divided by 12" to a foot / gear ratio * -> feet
    driveEnc.setVelocityConversionFactor(((4/12.0) * Math.PI) / (8.14 * 60));//circumference for 4" wheel divided by 12" to a foot / gear ratio * convert to seconds -> feet per second

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Speed" + positionID, getSpeed());
    SmartDashboard.putNumber("Distance" + positionID, getDistance());
    SmartDashboard.putNumber("Angle" + positionID, getAngle());
    // This method will be called once per scheduler run
  }

  public double getSpeed(){
    return driveEnc.getVelocity();
  }

  public double getDistance(){
    return Math.abs(driveEnc.getPosition());
  }
  public void resetDistance(){
    driveEnc.setPosition(0);
  }

  public double getAngle(){
    double angle = (dirEnc.getAbsolutePosition().getValue() * 360);
    return angle;
  }


  public void drivePod(double drive , double turn){
    driveMotor.set(drive);
    swerveMotor.set(turn);
  }




  
}
