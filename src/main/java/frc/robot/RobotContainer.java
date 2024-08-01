// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooteyThing;

public class RobotContainer {
  private double MaxSpeed = CommandSwerveDrivetrain.kDriveMaxSpeed; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = CommandSwerveDrivetrain.kTurnMaxSpeed; // 3/4 of a rotation per second max angular velocity

  private final CommandXboxController drivestick = new CommandXboxController(0);
  private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * CommandSwerveDrivetrain.kDriveDeadBand).withRotationalDeadband(MaxAngularRate * CommandSwerveDrivetrain.kTurnDeadBand) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
                                                             
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * CommandSwerveDrivetrain.kDriveDeadBand).withRotationalDeadband(MaxAngularRate * CommandSwerveDrivetrain.kTurnDeadBand)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final ShooteyThing m_shooter = ShooteyThing.getInstance();

  private void configureBindingsDrive() {
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(
        () -> {
          if (!drivetrain.getUseHeadingPID() || Math.abs(drivestick.getRightX()) > drivetrain.getTurnDeadBand()) {
            return drive.withVelocityX(-drivestick.getLeftY() * MaxSpeed)
                        .withVelocityY(-drivestick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-drivestick.getRightX() * MaxAngularRate);
          }
          else {
            return driveFacingAngle.withVelocityX(-drivestick.getLeftY() * MaxSpeed)
                  .withVelocityY(-drivestick.getLeftX() * MaxSpeed)
                  .withTargetDirection(Rotation2d.fromDegrees(drivetrain.getTargetHeadingDegrees()));
            
          }
        }
      )
    );

    new Trigger(
      () -> Math.abs(drivestick.getRightX()) > drivetrain.getTurnDeadBand()
    ).onTrue(
      new FunctionalCommand(
        () -> {},
        () -> {drivetrain.setTargetHeadingDegrees(drivetrain.getHeadingDegrees());}, 
        (interrupted) -> {drivetrain.setTargetHeadingDegrees(drivetrain.getHeadingDegrees());}, 
        () -> !drivetrain.isRotating() && Math.abs(drivestick.getRightX()) < drivetrain.getTurnDeadBand())
    );
    
    drivestick.back().onTrue(
      new InstantCommand(() -> drivetrain.toggleHeadingPID(), drivetrain)
    );

    drivestick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
    // drivestick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-drivestick.getLeftY(), -drivestick.getLeftX()))));

    drivestick.start().onTrue(drivetrain.resetHeadingCommand());

    drivestick.povLeft().onTrue(
      new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(90), drivetrain)
    );

    drivestick.povUp().onTrue(
      new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(0), drivetrain)
    );

    drivestick.povRight().onTrue(
      new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(-90), drivetrain)
    );

    drivestick.povDown().onTrue(
      new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(180), drivetrain)
    );

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureBindingsMech(){
    drivestick.leftBumper().onTrue(
      m_shooter.intakeCommand()
    );

    drivestick.x().onTrue(
      m_shooter.shootCommand()
    );

    drivestick.y().onTrue(
      m_shooter.ampCommand()
    );

    drivestick.b().onTrue(
      m_shooter.stopCommand()
    );
  }

  private void configureBindings(){
    this.configureBindingsDrive();
    this.configureBindingsMech();
  }

  public RobotContainer() {
    drivetrain.setSwerveRequest(this.driveFacingAngle);
    configureBindings();
  }

  public void loop(){
    MaxSpeed = drivetrain.getMaxDriveSpeed();
    MaxAngularRate = drivetrain.getMaxTurnSpeed() * Math.PI;

    drive.Deadband = drivetrain.getDriveDeadBand();
    drive.RotationalDeadband = drivetrain.getTurnDeadBand();

    driveFacingAngle.Deadband = drivetrain.getDriveDeadBand();
    driveFacingAngle.RotationalDeadband = drivetrain.getTurnDeadBand();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
