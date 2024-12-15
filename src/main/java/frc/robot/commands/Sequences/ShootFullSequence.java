// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.*;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.BCRRobotState.ShotMode;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootFullSequence extends SequentialCommandGroup {
  /** Creates a new ShootFullSequence. */
  public ShootFullSequence(AllianceSelection allianceSelection, DriveTrain driveTrain, Shooter shooter, Feeder feeder, Wrist wrist, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    //uses selector to get current shot mode
      new SelectCommand<>(
        Map.ofEntries(
          Map.entry(ShotMode.SPEAKER, new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log)),
          Map.entry(ShotMode.AMP, new ShootPieceAmp(feeder, robotState, log)),
          Map.entry(ShotMode.SHORT_PASS, new ShootPiece(ShooterConstants.shooterVelocityShortPassTop, ShooterConstants.shooterVelocityShortPassBottom, true, shooter, feeder, wrist, robotState, log)),
          Map.entry(ShotMode.FAR_PASS, new ShootPiece(ShooterConstants.shooterVelocityFarPassTop, ShooterConstants.shooterVelocityFarPassBottom, true, shooter, feeder, wrist, robotState, log)),
          Map.entry(ShotMode.VISION_FAR_PASS, new ShootPiecePassWithVision(true, allianceSelection, driveTrain, shooter, feeder, wrist, robotState, log)),
          Map.entry(ShotMode.VISION_MID_PASS, new ShootPiecePassWithVision(true, allianceSelection, driveTrain, shooter, feeder, wrist, robotState, log))
        ),
      robotState::getShotMode));
  }
}
