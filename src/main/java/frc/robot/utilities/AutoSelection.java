package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.commands.ChoreoFollower;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.Autos.*;
import frc.robot.commands.Sequences.IntakePiece;
import frc.robot.commands.Sequences.ShootPiece;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int NONE = 0;
	public static final int test = 1;
	public static final int shootOne = 2;
	public static final int CenterTwoPieceShoot = 3;
	public static final int SourceTwoPieceShoot = 4;
	public static final int AmpTwoPieceShoot = 5;
	public static final int AmpThreePieceShoot = 6;
	public static final int CenterSourceThreePieceShoot = 7;
	public static final int CenterFourPieceShoot = 8;
	public static final int SourceShootOnePiece = 9;
	public static final int AmpShootOnePiece = 10;
	public static final int SourceThreePieceCenter = 11;
	public static final int CenterFivePieceShootNextToEdge = 12;
	public static final int CenterFivePieceShootEdge = 13;
	public static final int AmpFourPieceCenter = 14;
	public static final int AlternetSourceThreeNoteCenter = 15;
	public static final int SourceFourNoteCenter = 16;
	public static final int SourceFifthNote = 17;
	public static final int SourceFifthNoteAndShoot = 18;
	public static final int SourceWallMobilityAuto = 19;
	public static final int AmpThreePieceCenter = 20;
	public static final int ChoreoTrajectory = 21;
	public static final int ChoreoTrajectoryWithFollower = 22;

	private final AllianceSelection allianceSelection;
	private final TrajectoryCache trajectoryCache;
	private SendableChooser<Integer> autoChooser = new SendableChooser<>();
	
	/**
	 * AutoSelection constructor for command group
	 * Sets up autoPlan widget 
	 */  	
	public AutoSelection(TrajectoryCache trajectoryCache, AllianceSelection allianceSelection, FileLog log) {
		this.trajectoryCache = trajectoryCache;
		this.allianceSelection = allianceSelection;

		// auto selections
		autoChooser.setDefaultOption("None", NONE);
		autoChooser.addOption("CenterTwoPieceShoot", CenterTwoPieceShoot);
		autoChooser.addOption("SourceTwoPieceShoot", SourceTwoPieceShoot);
		// autoChooser.addOption("AmpTwoPieceShoot", AmpTwoPieceShoot);
		// autoChooser.addOption("AmpThreePieceShoot", AmpThreePieceShoot);
		// autoChooser.addOption("OnePieceShoot", shootOne);
		autoChooser.addOption("CenterThreePieceShootAmp", CenterSourceThreePieceShoot);
		autoChooser.addOption("SourceShootOnePiece", SourceShootOnePiece);
		autoChooser.addOption("AmpShootOnePiece", AmpShootOnePiece);
		autoChooser.addOption("CenterFourPieceNearNoteAuto", CenterFourPieceShoot);
		autoChooser.addOption("CenterFivePieceShootNextToEdge", CenterFivePieceShootNextToEdge);
		autoChooser.addOption("CenterFivePieceShootEdge", CenterFivePieceShootEdge);
		autoChooser.addOption("AmpFourPieceCenter", AmpFourPieceCenter);
		autoChooser.addOption("SourceThreePieceCenter", SourceThreePieceCenter);
		autoChooser.addOption("AlternetSourceThreeNoteCenter", AlternetSourceThreeNoteCenter);
		autoChooser.addOption("SourceFourNoteCenter", SourceFourNoteCenter);
		autoChooser.addOption("SourceFifthNote", SourceFifthNote);
		autoChooser.addOption("SourceFifthNoteAndShoot", SourceFifthNoteAndShoot);
		autoChooser.addOption("SourceMobilityIntoSide", SourceWallMobilityAuto);
		autoChooser.addOption("AmpThreePieceCenter", AmpThreePieceCenter);
		autoChooser.addOption("ChoreoTrajectory", ChoreoTrajectory);
		autoChooser.addOption("ChoreoTrajectoryWithFollower", ChoreoTrajectoryWithFollower);




	
		// show auto selection widget on Shuffleboard
		SmartDashboard.putData("Autonomous routine", autoChooser);

		// show auto parameters on Shuffleboard
		SmartDashboard.putNumber("Autonomous delay", 0);
		// SmartDashboard.putBoolean("Autonomous use vision", false);
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard.
	 * This method is designed to be called at AutonomousInit by Robot.java.
	 * 
	 * @param driveTrain The driveTrain that will be passed to the auto command
	 * @param log        The filelog to write the logs to
	 * @return the command to run
	 */

	public Command getAutoCommand(Intake intake, Wrist wrist, Shooter shooter, Feeder feeder, DriveTrain driveTrain, BCRRobotState robotState, AutoFactory autoFactory, FileLog log) {
		Command autonomousCommandMain = null;

		
		// Get parameters from Shuffleboard
		int autoPlan = autoChooser.getSelected();
		log.writeLogEcho(true, "AutoSelect", "autoPlan",autoPlan);


		double waitTime = SmartDashboard.getNumber("Autonomous delay", 0);
		waitTime = MathUtil.clamp(waitTime, 0, 15);		// make sure autoDelay isn't negative and is only active during auto

		if (autoPlan == NONE) {
			// Starting position = facing drivers
			log.writeLogEcho(true, "AutoSelect", "run None");
			autonomousCommandMain = new DriveResetPose(180, false, driveTrain, log);
		}

		else if(autoPlan == test){
			log.writeLogEcho(true, "AutoSelect", "run Test");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kCoast, trajectoryCache.cache[TrajectoryCache.TrajectoryType.test.value].red, driveTrain, log);
		}

		else if(autoPlan == shootOne){
			log.writeLogEcho(true, "AutoSelect", "run One Piece Shoot");
			autonomousCommandMain = new ShootPiece(true, shooter, feeder, wrist, robotState, log);
		}

		else if(autoPlan == CenterTwoPieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Center Two Piece Shoot");
			autonomousCommandMain = new CenterTwoPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == SourceTwoPieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Source Two Piece Shoot");
			autonomousCommandMain = new SourceTwoPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AmpTwoPieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Amp Two Piece Shoot");
			autonomousCommandMain = new AmpTwoPieceShoot(intake, shooter, driveTrain, feeder, wrist, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == CenterSourceThreePieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Source Center Three Piece Shoot");
			autonomousCommandMain = new CenterThreePieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AmpThreePieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Amp Three Piece Shoot");
			autonomousCommandMain = new AmpThreePieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}
		
		else if(autoPlan == CenterFourPieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Center Four Piece Shoot");
			autonomousCommandMain = new CenterFourPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == SourceShootOnePiece){
			log.writeLogEcho(true, "AutoSelect", "run Source One Piece Shoot");
			autonomousCommandMain = new SourceShootOnePiece(intake, wrist, shooter, driveTrain, feeder, robotState, allianceSelection, log);
		}

        else if(autoPlan == AmpShootOnePiece){
			log.writeLogEcho(true, "AutoSelect", "run Amp One Piece Shoot");
			autonomousCommandMain = new AmpShootOnePiece(intake, wrist, shooter, driveTrain, feeder, robotState, allianceSelection, log);
		}
		
        else if(autoPlan == SourceThreePieceCenter){
			log.writeLogEcho(true, "AutoSelect", "run Source Three Piece Center");
			autonomousCommandMain = new SourceThreeNoteCenter(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == CenterFivePieceShootNextToEdge){
			log.writeLogEcho(true, "AutoSelect", "run Center Five Piece Shoot Next To Edge");
			autonomousCommandMain = new CenterFivePieceShootNextToEdge(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}
		else if(autoPlan == CenterFivePieceShootEdge){
			log.writeLogEcho(true, "AutoSelect", "run Center Five Piece Shoot Edge");
			autonomousCommandMain = new CenterFivePieceShootEdge(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AmpFourPieceCenter){
			log.writeLogEcho(true, "AutoSelect", "run Amp Four Piece Center");
			autonomousCommandMain = new AmpThreePieceCenter(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		
		}	
		
		else if(autoPlan == AlternetSourceThreeNoteCenter){
			log.writeLogEcho(true, "AutoSelect", "run Alternet SourceThreeNoteCenter");
			autonomousCommandMain = new AlternetSourceThreeNoteCenter(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == SourceFourNoteCenter){
			log.writeLogEcho(true, "AutoSelect", "run Source Four Note Center");
			autonomousCommandMain = new SourceFourNoteCenter(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == SourceFifthNote){
			log.writeLogEcho(true, "AutoSelect", "run Source One Note Mobility");
			autonomousCommandMain = new SourceOnePieceDriveToFifthNote(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == SourceFifthNoteAndShoot){
			log.writeLogEcho(true, "AutoSelect", "run Source Two Notes With Fifth note");
			autonomousCommandMain = new SourceTwoPieceFifthNoteShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == SourceWallMobilityAuto){
			log.writeLogEcho(true, "AutoSelect", "run Source Auto Into the Side");
			autonomousCommandMain = new SourceWallMobilityAuto(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if (autoPlan == AmpThreePieceCenter) {
			log.writeLogEcho(true, "AutoSelect", "run Amp Three Piece");
			autonomousCommandMain = new AmpThreePieceCenter(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if (autoPlan == ChoreoTrajectory){
			log.writeLogEcho(true, "AutoSelect", "run Choreo Trajectory");
			autonomousCommandMain = new SequentialCommandGroup(
				autoFactory.resetOdometry("Test-Path"),
				autoFactory.trajectoryCmd("Test-Path"));
		}

		else if (autoPlan == ChoreoTrajectoryWithFollower){
			log.writeLogEcho(true, "AutoSelect", "run Choreo Trajectory With Follower command");
			Optional<Trajectory<SwerveSample>> loadTrajectory = Choreo.loadTrajectory("Test-Path");
			Trajectory<SwerveSample> Trajectory = loadTrajectory.get();
			if(!loadTrajectory.isEmpty()){
				autonomousCommandMain = new SequentialCommandGroup(
					new DriveResetPose(() -> ((allianceSelection.getAlliance() == Alliance.Red) ? Trajectory.getInitialPose(true).get() : Trajectory.getInitialPose(false).get()), false, driveTrain, log),
					new ChoreoFollower(Trajectory,
						new PIDController(Constants.TrajectoryConstants.kPXController, 0.0, 0.0), 
						new PIDController(Constants.TrajectoryConstants.kPYController, 0.0, 0.0), 
						new PIDController(Constants.TrajectoryConstants.kPThetaController, 0.0, 0.0), 
						(speeds) -> driveTrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true, false),
						() -> driveTrain.getPose(),
						() -> allianceSelection.getAlliance() == Alliance.Red, 
						driveTrain, 
						log)
				);
			}
		}

		else if (autonomousCommandMain == null) {
			log.writeLogEcho(true, "AutoSelect", "No autocommand found");
			autonomousCommandMain = new WaitCommand(1);
		}


		// Add auto wait time before the main auto command
		Command autonomousCommand = new SequentialCommandGroup(
			new WaitCommand(waitTime),
			autonomousCommandMain
		);

		return autonomousCommand;
	}

}