package org.firstinspires.ftc.teamcode.globals;

import static com.qualcomm.robotcore.hardware.configuration.LynxConstants.EXPANSION_HUB_PRODUCT_NUMBER;
import static com.qualcomm.robotcore.hardware.configuration.LynxConstants.SERVO_HUB_PRODUCT_NUMBER;

import com.outoftheboxrobotics.photoncore.PhotonCore;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.SensorDigitalDevice;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.TelemetryData;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.Camera;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;

import dev.nullftc.profiler.Profiler;
import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;


public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    private static final Robot instance = new Robot();
    public static Robot getInstance() {
        return instance;
    }

    public Profiler profiler;
    public File file;

    public LynxModule controlHub;
    public LynxModule expansionHub;
    public LynxModule servoHub;
    public ArrayList<LynxModule> hubs;
    public VoltageSensor voltageSensor;
    private double cachedVoltage;
    private ElapsedTime voltageTimer;

    public boolean readyToLaunch = false;

    public MotorEx FRmotor;
    public MotorEx FLmotor;
    public MotorEx BLmotor;
    public MotorEx BRmotor;

    public MotorGroup intakeMotors;

    public MotorGroup launchMotors;
    public Motor.Encoder launchEncoder;

    public CRServoEx FRswervo;
    public CRServoEx FLswervo;
    public CRServoEx BLswervo;
    public CRServoEx BRswervo;

    public CRServoGroup turretServos;
    public AbsoluteAnalogEncoder turretEncoder;

    public ServoEx intakePivotServo;
    public ServoEx hoodServo;
    public ServoEx rampServo;

    public Limelight3A limelight;

    public GoBildaPinpointDriver pinpoint;
//    public IMU imu;

    public Drive drive;
    public Camera camera;

    public SensorDigitalDevice frontDistanceSensor;
    public SensorDigitalDevice backDistanceSensor;
    public AnalogInput distanceSensor;

    public void init(HardwareMap hwMap) {
        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logsFolder.exists()) logsFolder.mkdirs();

        long timestamp = System.currentTimeMillis();
        file = new File(logsFolder, "profiler-" + timestamp + ".csv");

        profiler = Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(file))
                .debugLog(false) // Log EVERYTHING
                .build();

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        // Hardware
        voltageSensor = hwMap.voltageSensor.iterator().next();

        FRmotor = new MotorEx(hwMap, "FRM").setCachingTolerance(0.01);
        FLmotor = new MotorEx(hwMap, "FLM").setCachingTolerance(0.01);
        BLmotor = new MotorEx(hwMap, "BLM").setCachingTolerance(0.01);
        BRmotor = new MotorEx(hwMap, "BRM").setCachingTolerance(0.01);

        FRmotor.setRunMode(Motor.RunMode.RawPower);
        FLmotor.setRunMode(Motor.RunMode.RawPower);
        BLmotor.setRunMode(Motor.RunMode.RawPower);
        BRmotor.setRunMode(Motor.RunMode.RawPower);


        FRswervo = new CRServoEx(hwMap, "FRS", new AbsoluteAnalogEncoder(hwMap, "FRE")
                .zero(FR_ENCODER_OFFSET).setReversed(true), CRServoEx.RunMode.RawPower)
                .setCachingTolerance(0.02);
        FLswervo = new CRServoEx(hwMap, "FLS", new AbsoluteAnalogEncoder(hwMap, "FLE")
                .zero(FL_ENCODER_OFFSET).setReversed(true), CRServoEx.RunMode.RawPower)
                .setCachingTolerance(0.02);
        BLswervo = new CRServoEx(hwMap, "BLS", new AbsoluteAnalogEncoder(hwMap, "BLE")
                .zero(BL_ENCODER_OFFSET).setReversed(true), CRServoEx.RunMode.RawPower)
                .setCachingTolerance(0.02);
        BRswervo = new CRServoEx(hwMap, "BRS", new AbsoluteAnalogEncoder(hwMap, "BRE")
                .zero(BR_ENCODER_OFFSET).setReversed(true), CRServoEx.RunMode.RawPower)
                .setCachingTolerance(0.02);
        FRswervo.setInverted(true);
        FLswervo.setInverted(true);
        BLswervo.setInverted(true);
        BRswervo.setInverted(true);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint")
                .setOffsets(-100, -40, DistanceUnit.MM)
                .setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
                .setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC)
                .resetPosAndIMU()
                .setPosition(Pose2d.convertToPose2D(END_POSE, DistanceUnit.INCH, AngleUnit.RADIANS))
                .setBulkReadScope(GoBildaPinpointDriver.Register.X_POSITION, GoBildaPinpointDriver.Register.Y_POSITION, GoBildaPinpointDriver.Register.H_ORIENTATION);


//        frontDistanceSensor = new SensorDigitalDevice(hwMap, "frontDistanceSensor", FRONT_DISTANCE_THRESHOLD);
//        backDistanceSensor = new SensorDigitalDevice(hwMap, "backDistanceSensor", BACK_DISTANCE_THRESHOLD);

        //distanceSensor = hwMap.get(AnalogInput.class, "distanceSensor");

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250);
        limelight.pipelineSwitch(4);
        limelight.start();

        // Subsystems
        drive = new Drive();

        // vision
//        camera = new Camera(hwMap);

         CommandScheduler configurations;
        setBulkReading(hwMap, LynxModule.BulkCachingMode.MANUAL);

        for (LynxModule hub : hwMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                controlHub = hub;
            } else if (!hub.isParent() && hub.getRevProductNumber() == (EXPANSION_HUB_PRODUCT_NUMBER)) {
                expansionHub = hub;
            } else if (!hub.isParent() && hub.getRevProductNumber() == (SERVO_HUB_PRODUCT_NUMBER)) {
                servoHub = hub;
            }
        }

        register(drive);

        if (OP_MODE_TYPE.equals(OpModeType.AUTO)) {
            initHasMovement();
        }
    }

    public void initHasMovement() {
        drive.init();
        //camera.initHasMovement();
    }
    
    public double getVoltage() {
//        return 12;
        // this is chopped for loop times
        if (voltageTimer == null) {
            cachedVoltage = voltageSensor.getVoltage();
        } else if (voltageTimer.milliseconds() > (1.0 / VOLTAGE_SENSOR_POLLING_RATE) * 1000) {
            cachedVoltage = voltageSensor.getVoltage();
        }
        if (((Double) cachedVoltage).isNaN() || cachedVoltage == 0) {
            cachedVoltage = 12;
        }
        return cachedVoltage;
    }

    public void exportProfiler(File file) {
        RobotLog.i("Starting async profiler export to: " + file.getAbsolutePath());

        Thread exportThread = new Thread(() -> {
            try {
                profiler.export();
                profiler.shutdown();
            } catch (Exception e) {
                Log.e("An error occurred", e.toString());
                Log.e(e.toString(), Arrays.toString(e.getStackTrace()));
            }
        });

        exportThread.setDaemon(true);
        exportThread.start();
    }

    /* Not needed now that we have pinpoint
    public void initializeImu(HardwareMap hardwareMap) {
        // IMU orientation
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
     */

    public void updateLoop(TelemetryData telemetryData) {
        CommandScheduler.getInstance().run();
        telemetryData.update();
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
    }
}