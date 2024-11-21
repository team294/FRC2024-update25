// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.BCRRobotState.ShotMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToAmp extends SequentialCommandGroup {
  /** Creates a new DriveToAmp. */
  public DriveToAmp(AllianceSelection allianceSelection, Intake intake, Feeder feeder, Wrist wrist, DriveTrain driveTrain, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new DriveToPose(() -> allianceSelection.getAmpPosInitial(), .25, 10, driveTrain, log),
      new ParallelCommandGroup(
        new ConditionalCommand(
          new DriveToPose(CoordType.kAbsolute, 90, driveTrain, log),
          new DriveToPose(CoordType.kAbsolute, -90, driveTrain, log), 
          () -> allianceSelection.getAlliance() == Alliance.Red
        ),
        new IntakeStop(intake, log),
        new ShotModeSet(ShotMode.AMP, robotState, log),
        new RobotStateSetIdle(robotState, feeder, log)
      ),
      //new DriveToPose(CoordType.kAbsolute, new Rotation2d(Math.toRadians(90)), driveTrain, log),
      new ParallelDeadlineGroup(
        new WristSetAngle(WristAngle.ampShot, wrist, log),
        new DriveToPose(() -> allianceSelection.getAmpPosInitial(), .25, 10, driveTrain, log)
      ),
      new DriveToPose(allianceSelection.getAmpPos(), driveTrain, log)
    );
  }
}
