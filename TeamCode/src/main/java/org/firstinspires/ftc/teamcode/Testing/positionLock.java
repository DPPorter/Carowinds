package org.firstinspires.ftc.teamcode.Testing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
@Configurable
@TeleOp
public class positionLock extends OpMode {
    public DcMotorEx turretMotor;
    public DcMotorEx flywheelMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx popperMotor;

    public Servo popServo;
    public Servo hoodServo;
    public Servo topServo;

    public Limelight3A limelight;
    public Servo underglow;
    public DigitalChannel intakeBeam;
    public DigitalChannel outtakeBeam;


    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;


    PIDFCoefficients pidVariables = new PIDFCoefficients(250, 0, 0, 17.7, MotorControlAlgorithm.PIDF);

    boolean lock = false;

    ElapsedTime lockTimer = new ElapsedTime();

    @Override
    public void init() {

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.4);

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "spinMotor");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidVariables);
        flywheelMotor.setVelocity(0);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);

        popperMotor = hardwareMap.get(DcMotorEx.class, "popperMotor");
        popperMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        popperMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        popperMotor.setPower(0);


        popServo = hardwareMap.get(Servo.class, "transferServo");
        popServo.setPosition(popServo.getPosition());

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        topServo = hardwareMap.get(Servo.class, "topServo");
        topServo.setPosition(topServo.getPosition());


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);

        underglow = hardwareMap.get(Servo.class, "underglow");
        underglow.setPosition(underglow.getPosition());

        intakeBeam = hardwareMap.get(DigitalChannel.class, "intakeBeam");
        intakeBeam.setMode(DigitalChannel.Mode.INPUT);

        outtakeBeam = hardwareMap.get(DigitalChannel.class, "outtakeBeam");
        outtakeBeam.setMode(DigitalChannel.Mode.INPUT);



        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(54, 94))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        drivetrain();
        llReset();






        telemetry.addData("locked", lock);


                telemetry.addData("Status", "Initialized. Connect a PS4/PS5 Controller!");
                telemetry.update();

                    // 1. Check if finger 1 is pressing the touchpad
                    boolean finger1Down = gamepad1.touchpad_finger_1;

                    if (finger1Down) {
                        // Read the position where the finger is sliding
                        float finger1X = gamepad1.touchpad_finger_1_x;
                        float finger1Y = gamepad1.touchpad_finger_1_y;

                        telemetry.addData("Finger 1 Status", "TOUCHING");
                        telemetry.addData("Finger 1 X (Left/Right)", "%.2f", finger1X);
                        telemetry.addData("Finger 1 Y (Up/Down)", "%.2f", finger1Y);
                    } else {
                        telemetry.addData("Finger 1 Status", "No Touch");
                    }

                    // 2. Check if finger 2 is also pressing (Multi-touch support)
                    if (gamepad1.touchpad_finger_2) {
                        telemetry.addData("Finger 2 X", "%.2f", gamepad1.touchpad_finger_2_x);
                        telemetry.addData("Finger 2 Y", "%.2f", gamepad1.touchpad_finger_2_y);
                    }

                    // 3. Check for a physical click (pressing down hard on the touchpad like a button)
                    if (gamepad1.touchpad) {
                        telemetry.addData("Touchpad Button", "CLICKED!");
                    } else {
                        telemetry.addData("Touchpad Button", "Not Clicked");
                    }



    }

    Pose lockPose = new Pose(0,0,0);
    private void drivetrain(){
        if((Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x)) < 0.1) {
            if(follower.getVelocity().getMagnitude() < 6 && Math.abs(follower.getAngularVelocity()) < 0.09)
                lock = true;
        }else{
            if(lock)
                follower.startTeleopDrive();
            lock = false;
        }

        if(!lock){
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
            lockPose = follower.getPose();
        }else{
            follower.holdPoint(lockPose, false);
        }
    }

    private void llReset(){
        LLResult results = limelight.getLatestResult();

        Pose2D ftcPose2d = new Pose2D(DistanceUnit.INCH, (results.getBotpose().getPosition().x * 39.3701), (results.getBotpose().getPosition().y * 39.3701), AngleUnit.RADIANS, AngleUnit.normalizeRadians(results.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)));

        Pose ftcStandard = PoseConverter.pose2DToPose(ftcPose2d, InvertedFTCCoordinates.INSTANCE);
        Pose current = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);


        if(results.getBotposeAvgDist() != 0 && Math.abs(results.getTx()) < 10) follower.setPose(current);
    }
}

