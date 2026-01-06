package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.globals.Constants.END_POSE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SwervePathingOpMode.FOLLOW_PREPROGRAMMED_PATHS;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SwervePathingOpMode.headingTarget;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SwervePathingOpMode.xTarget;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SwervePathingOpMode.yTarget;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.drivebase.swerve.coaxial.CoaxialSwerveModule;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.commandbase.commands.DriveTo;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.globals.Robot;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Config
@TeleOp(name = "SwerveTeleOp")
public class SwerveTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;
    public ElapsedTime timer;
    private GoBildaPinpointDriver pinpoint;
//    private MotorEx intakeLeft = new MotorEx(hardwareMap, "intake left", Motor.GoBILDA.RPM_435);
//    private MotorEx intakeRight = new MotorEx(hardwareMap, "intake right", Motor.GoBILDA.RPM_435);
//    private MotorGroup intake = new MotorGroup(intakeLeft, intakeRight);
//    private MotorEx shooterLeft = new MotorEx(hardwareMap, "shooter left", Motor.GoBILDA.BARE);
//    private MotorEx shooterRight = new MotorEx(hardwareMap, "shooter right", Motor.GoBILDA.BARE);
//    private MotorGroup shooter = new MotorGroup(shooterLeft, shooterRight);
//    private CRServoEx hoodLeft = new CRServoEx(hardwareMap, "hood left");
//    private CRServoEx hoodRight = new CRServoEx(hardwareMap, "hood right");
//    private CRServoGroup hood = new CRServoGroup(hoodLeft, hoodRight);

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
    private final Robot robot = Robot.getInstance();

    public ArrayList<Pose2d> pathPoses;

    private Limelight3A limelight;
    private IMU imu;
    public static double xTarget = 1;
    public static double yTarget = 1;
    public static double headingTarget = 0;
    public void generatePath() {
        pathPoses = new ArrayList<Pose2d>();

        pathPoses.add(new Pose2d(0, 0, 0)); // Starting Pose
        pathPoses.add(new Pose2d(24, 24, Math.PI/2)); // Line 1
        pathPoses.add(new Pose2d(0, 0, 0)); // Line 2
    }

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint")
                .setOffsets(-100, -40, DistanceUnit.MM)
                .setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
                .setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC)
                .resetPosAndIMU()
                .setPosition(Pose2d.convertToPose2D(END_POSE, DistanceUnit.INCH, AngleUnit.RADIANS))
                .setBulkReadScope(GoBildaPinpointDriver.Register.X_POSITION, GoBildaPinpointDriver.Register.Y_POSITION, GoBildaPinpointDriver.Register.H_ORIENTATION);

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Driver controls
        // Reset heading
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.drive.setPose(new Pose2d()))
        );
        limelight.start();
//        intakeRight.setInverted(true);
//        shooterRight.setInverted(true);
//        hoodRight.setInverted(true);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2();
            double x = botpose.getPosition().x;
            double y = botpose.getPosition().y;
            double heading = botpose.getOrientation().getYaw();
            Pose2D pinpointPose = new Pose2D (DistanceUnit.MM, x, y, AngleUnit.DEGREES, heading);
//            pinpoint.resetPosAndIMU();
//            pinpoint.setPosition(pinpointPose);
        }
        telemetryData.addData("ta", llResult.getTa());
        telemetryData.addData("tx", llResult.getTx());
        telemetryData.addData("ty", llResult.getTy());
        telemetryData.addData("pinpont x", pinpoint.getEncoderX());
        telemetryData.addData("pinpoint y", pinpoint.getEncoderY());

        // Update any constants that are being updated by FTCDash
        for (CoaxialSwerveModule module : robot.drive.swerve.getModules()) {
            module.setSwervoPIDF(Constants.SWERVO_PIDF_COEFFICIENTS);
        }

        while (gamepad1.a) {
//            if (pathPoses == null)
//                generatePath();
//            robot.drive.setPose(pathPoses.get(0));
//            schedule(
//                    new InstantCommand(),
//                    new ConditionalCommand(
//                            new SequentialCommandGroup(
//                                new DriveTo(pathPoses.get(1)),
//                                new DriveTo(pathPoses.get(2))
//                            ),
//                            new RunCommand(
//                                    () -> schedule(new DriveTo(new Pose2d(xTarget, yTarget, new Rotation2d(headingTarget))))
//                            ),
//                            () -> FOLLOW_PREPROGRAMMED_PATHS
//                    )
//            );
            new InstantCommand(
                    () -> schedule(new DriveTo(new Pose2d(xTarget, yTarget, new Rotation2d(headingTarget))))
            );
        };

//        if (gamepad1.left_trigger > 0.8){
//            intake.set(-50);
//        }
//        else {
//            intake.set(0);
//        };

//        if (gamepad1.right_trigger > 0.8){
//            intake.set(-50);
//            shooter.set(100);
//        }
//        else {
//            intake.set(0);
//            shooter.set(0);
//        };

//        while (gamepad1.b){
//            intake.set(5);
//        };

        // Drive the robot
        double minSpeed = 0.3; // As a fraction of the max speed of the robot
        double speedMultiplier = minSpeed + (1 - minSpeed) * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        robot.drive.swerve.updateWithTargetVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        driver.getLeftY() * Constants.MAX_DRIVE_VELOCITY * speedMultiplier,
                        -driver.getLeftX() * Constants.MAX_DRIVE_VELOCITY * speedMultiplier,
                        -driver.getRightX() * Constants.MAX_ANGULAR_VELOCITY * speedMultiplier,
                        robot.drive.getPose().getRotation()
                )
        );

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("Heading", robot.drive.getPose().getHeading());
        telemetryData.addData("Robot Pose", robot.drive.getPose());

        telemetryData.addData("Target Chassis Velocity", robot.drive.swerve.getTargetVelocity());
        telemetryData.addData("FR Module", robot.drive.swerve.getModules()[0].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[0].getPowerTelemetry());
        telemetryData.addData("FL Module", robot.drive.swerve.getModules()[1].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[1].getPowerTelemetry());
        telemetryData.addData("BL Module", robot.drive.swerve.getModules()[2].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[2].getPowerTelemetry());
        telemetryData.addData("BR Module", robot.drive.swerve.getModules()[3].getTargetVelocity() + " | " + robot.drive.swerve.getModules()[3].getPowerTelemetry());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        robot.updateLoop(telemetryData);
    }

    @Override
    public void end() {
        Constants.END_POSE = robot.drive.getPose();
        robot.exportProfiler(robot.file);
        telemetryData.update();
    }
}