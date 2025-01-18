package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.Autos.*;
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

	public enum AutoSelectionOption {
		NONE("None", 0),
		test("Test", 1),
		shootOne("OnePieceShoot", 2),
		CenterTwoPieceShoot("CenterTwoPieceShoot", 3),
		SourceTwoPieceShoot("SourceTwoPieceShoot", 4),
		AmpTwoPieceShoot("AmpTwoPieceShoot", 5),
		AmpThreePieceShoot("AmpThreePieceShoot", 6),
		CenterSourceThreePieceShoot("CenterThreePieceShootAmp", 7),
		CenterFourPieceShoot("CenterFourPieceNearNoteAuto", 8),
		SourceShootOnePiece("SourceShootOnePiece", 9),
		AmpShootOnePiece("AmpShootOnePiece", 10),
		SourceThreePieceCenter("SourceThreePieceCenter", 11),
		CenterFivePieceShootNextToEdge("CenterFivePieceShootNextToEdge", 12),
		CenterFivePieceShootEdge("CenterFivePieceShootEdge", 13),
		AmpFourPieceCenter("AmpFourPieceCenter", 14),
		AlternetSourceThreeNoteCenter("AlternetSourceThreeNoteCenter", 15),
		SourceFourNoteCenter("SourceFourNoteCenter", 16),
		SourceFifthNote("SourceFifthNote", 17),
		SourceFifthNoteAndShoot("SourceFifthNoteAndShoot", 18),
		SourceWallMobilityAuto("SourceMobilityIntoSide", 19),
		AmpThreePieceCenter("AmpThreePieceCenter", 20);

			@SuppressWarnings({"MemberName", "PMD.SingularField"})	
			public final String name;
		public final int value;
		AutoSelectionOption(String name, int value) { this.name = name; this.value = value; }

	}


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
		autoChooser.setDefaultOption(AutoSelectionOption.NONE.name, AutoSelectionOption.NONE.value);
		for (AutoSelectionOption option : AutoSelectionOption.values()) {
			if (option == AutoSelectionOption.NONE) continue;
			autoChooser.addOption(option.name, option.value);
		}




	
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

	public Command getAutoCommand(Intake intake, Wrist wrist, Shooter shooter, Feeder feeder, DriveTrain driveTrain, BCRRobotState robotState, FileLog log) {
		Command autonomousCommandMain = null;

		// Get parameters from Shuffleboard
		int autoPlan = autoChooser.getSelected();
		log.writeLogEcho(true, "AutoSelect", "autoPlan",autoPlan);


		double waitTime = SmartDashboard.getNumber("Autonomous delay", 0);
		waitTime = MathUtil.clamp(waitTime, 0, 15);		// make sure autoDelay isn't negative and is only active during auto

		if (autoPlan == AutoSelectionOption.NONE.value) {
			// Starting position = facing drivers
			log.writeLogEcho(true, "AutoSelect", "run None");
			autonomousCommandMain = new DriveResetPose(180, false, driveTrain, log);
		}

		else if(autoPlan == AutoSelectionOption.test.value){
			log.writeLogEcho(true, "AutoSelect", "run Test");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kCoast, trajectoryCache.cache[TrajectoryCache.TrajectoryType.test.value].red, driveTrain, log);
		}

		else if(autoPlan == AutoSelectionOption.shootOne.value){
			log.writeLogEcho(true, "AutoSelect", "run One Piece Shoot");
			autonomousCommandMain = new ShootPiece(true, shooter, feeder, wrist, robotState, log);
		}

		else if(autoPlan == AutoSelectionOption.CenterTwoPieceShoot.value){
			log.writeLogEcho(true, "AutoSelect", "run Center Two Piece Shoot");
			autonomousCommandMain = new CenterTwoPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AutoSelectionOption.SourceTwoPieceShoot.value){
			log.writeLogEcho(true, "AutoSelect", "run Source Two Piece Shoot");
			autonomousCommandMain = new SourceTwoPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AutoSelectionOption.AmpTwoPieceShoot.value){
			log.writeLogEcho(true, "AutoSelect", "run Amp Two Piece Shoot");
			autonomousCommandMain = new AmpTwoPieceShoot(intake, shooter, driveTrain, feeder, wrist, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AutoSelectionOption.CenterSourceThreePieceShoot.value){
			log.writeLogEcho(true, "AutoSelect", "run Source Center Three Piece Shoot");
			autonomousCommandMain = new CenterThreePieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AutoSelectionOption.AmpThreePieceShoot.value){
			log.writeLogEcho(true, "AutoSelect", "run Amp Three Piece Shoot");
			autonomousCommandMain = new AmpThreePieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}
		
		else if(autoPlan == AutoSelectionOption.CenterFourPieceShoot.value){
			log.writeLogEcho(true, "AutoSelect", "run Center Four Piece Shoot");
			autonomousCommandMain = new CenterFourPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AutoSelectionOption.SourceShootOnePiece.value){
			log.writeLogEcho(true, "AutoSelect", "run Source One Piece Shoot");
			autonomousCommandMain = new SourceShootOnePiece(intake, wrist, shooter, driveTrain, feeder, robotState, allianceSelection, log);
		}

        else if(autoPlan == AutoSelectionOption.AmpShootOnePiece.value){
			log.writeLogEcho(true, "AutoSelect", "run Amp One Piece Shoot");
			autonomousCommandMain = new AmpShootOnePiece(intake, wrist, shooter, driveTrain, feeder, robotState, allianceSelection, log);
		}
		
        else if(autoPlan == AutoSelectionOption.SourceThreePieceCenter.value){
			log.writeLogEcho(true, "AutoSelect", "run Source Three Piece Center");
			autonomousCommandMain = new SourceThreeNoteCenter(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AutoSelectionOption.CenterFivePieceShootNextToEdge.value){
			log.writeLogEcho(true, "AutoSelect", "run Center Five Piece Shoot Next To Edge");
			autonomousCommandMain = new CenterFivePieceShootNextToEdge(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}
		else if(autoPlan == AutoSelectionOption.CenterFivePieceShootEdge.value){
			log.writeLogEcho(true, "AutoSelect", "run Center Five Piece Shoot Edge");
			autonomousCommandMain = new CenterFivePieceShootEdge(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AutoSelectionOption.AmpFourPieceCenter.value){
			log.writeLogEcho(true, "AutoSelect", "run Amp Four Piece Center");
			autonomousCommandMain = new AmpThreePieceCenter(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		
		}	
		
		else if(autoPlan == AutoSelectionOption.AlternetSourceThreeNoteCenter.value){
			log.writeLogEcho(true, "AutoSelect", "run Alternet SourceThreeNoteCenter");
			autonomousCommandMain = new AlternetSourceThreeNoteCenter(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AutoSelectionOption.SourceFourNoteCenter.value){
			log.writeLogEcho(true, "AutoSelect", "run Source Four Note Center");
			autonomousCommandMain = new SourceFourNoteCenter(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AutoSelectionOption.SourceFifthNote.value){
			log.writeLogEcho(true, "AutoSelect", "run Source One Note Mobility");
			autonomousCommandMain = new SourceOnePieceDriveToFifthNote(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AutoSelectionOption.SourceFifthNoteAndShoot.value){
			log.writeLogEcho(true, "AutoSelect", "run Source Two Notes With Fifth note");
			autonomousCommandMain = new SourceTwoPieceFifthNoteShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AutoSelectionOption.SourceWallMobilityAuto.value){
			log.writeLogEcho(true, "AutoSelect", "run Source Auto Into the Side");
			autonomousCommandMain = new SourceWallMobilityAuto(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if (autoPlan == AutoSelectionOption.AmpThreePieceCenter.value) {
			log.writeLogEcho(true, "AutoSelect", "run Amp Three Piece");
			autonomousCommandMain = new AmpThreePieceCenter(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
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