package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class Launcher extends OpMode {
    private DcMotor lMotor;
    private DcMotor rMotor;
    private Servo servo;
    private double targetPosition; // target position for servo when shooting
    private Limelight3A limelight;
    private IMU imu;
    private final double MOTOR_SPEED = 0.575;
    private boolean stopMotors = false;
    private double distance; // horizontal distance from the april tag
    private final double LLTOTAG_Y = 19; // how much higher the april tag is to the limelight on the 2D y-axis
    private final double MOUNT_ANGLE = 0; // limelight mount angle

    @Override
    public void init() {
        // change deviceNames according to robot

        lMotor = hardwareMap.get(DcMotor.class, "left_flywheel");
        rMotor = hardwareMap.get(DcMotor.class, "right_flywheel");

        lMotor.setDirection(DcMotorSimple.Direction.REVERSE); // or rMotor, whichever makes it turn the right direction

        servo = hardwareMap.get(Servo.class, "servo");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

    }

    public double determineAngle(double distX) {
        return distX + 0; // change this to the curve determined from the regression model,
        // return in degrees
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();

            distance = LLTOTAG_Y/(Math.tan((llResult.getTy() + MOUNT_ANGLE) * Math.PI/180));

            targetPosition = 1 - ((determineAngle(distance) - 45)/35);

            if (targetPosition > 1) {
                targetPosition = 1;
            }
            else if (targetPosition < 0) {
                targetPosition = 0;
            }
            // plug in distance as x and angle as y in a curve regression model, set servo angle according to result

            servo.setPosition(targetPosition);
        }

        if (lMotor.getPower() != MOTOR_SPEED || rMotor.getPower() != MOTOR_SPEED && !stopMotors) {
            lMotor.setPower(MOTOR_SPEED);
            rMotor.setPower(MOTOR_SPEED);
        }

        if (stopMotors) { // kill switch
            lMotor.setPower(0);
            rMotor.setPower(0);
        }
    }
}
