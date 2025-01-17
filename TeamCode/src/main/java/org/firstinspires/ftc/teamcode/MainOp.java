package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@TeleOp
public class MainOp extends OpMode {

    private ColorSensor cSensor2;
    private ColorSensor cSensor;
    private DistanceSensor sensor;
    private DcMotorEx BLMotor;
    private DcMotorEx BRMotor;
    private DcMotorEx FLMotor;
    private DcMotorEx FRMotor;

    private double joystickY;
    private double joystickX;

    private double kp = .5;

    private int ticksPerRev = 28;
    private int wheelDiameterMM = 90;
    private double mmToInchConversion = 25.4;
    private double wheelDiameterIn = wheelDiameterMM / mmToInchConversion;
    private double gearRatio = 13.1;
    // i believe this is wrong
    private double ticksPerInch = ((Math.PI * wheelDiameterIn) / gearRatio) / ticksPerRev;

    private double closePosIn = 1.5;
    private int farPosIn = 10;

    private boolean drive = true;
    private boolean close = true;

    @Override
    public void init() {
        cSensor = hardwareMap.get(ColorSensor.class, "cSensor");
        cSensor2 = hardwareMap.get(ColorSensor.class, "cSensor2");
        sensor = hardwareMap.get(DistanceSensor.class, "sensor");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BRMotor");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotorEx.class, "FRMotor");

        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            drive = true;
        } else if (gamepad1.a && sensor.getDistance(DistanceUnit.INCH) < 9) {
            drive = false;
            close = true;
        } // else if (gamepad1.b && sensor.getDistance(DistanceUnit.INCH) < 8) {
//            drive = false;
//            close = false;
//        }

        if (drive || sensor.getDistance(DistanceUnit.INCH) > 9){

            drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        } else if (close){
            // go in direction to match distance sensor value to close value
            if (closePosIn - .1 > sensor.getDistance(DistanceUnit.INCH) - 2){
                setPowers(.08 * Math.abs(closePosIn - sensor.getDistance(DistanceUnit.INCH) + 2));
            } else if (closePosIn + .1 < sensor.getDistance(DistanceUnit.INCH) - 2){
                setPowers(-.08 * Math.abs(closePosIn - sensor.getDistance(DistanceUnit.INCH) + 2));
            } else {
                setPowers(0);
                drive = true;
            }
        } //else {
//            // go in direction to match distance sensor value to far value
//            if (farPosIn - .1 > sensor.getDistance(DistanceUnit.INCH) - 2){
//                setPowers(.05 * Math.abs(farPosIn - sensor.getDistance(DistanceUnit.INCH) - 2));
//            } else if (farPosIn + .1 < sensor.getDistance(DistanceUnit.INCH) - 2){
//                setPowers(-.05 * Math.abs(farPosIn - sensor.getDistance(DistanceUnit.INCH) - 2));
//            } else {
//                setPowers(0);
//                drive = true;
//            }
//        }

        telemetry.addData("Sensor Value: ", sensor.getDistance(DistanceUnit.INCH) - 2);
//        telemetry.addData("BLMotor Encoder Pos: ", BLMotor.getCurrentPosition());
//        telemetry.addData("Target Position: ", BLMotor.getTargetPosition());
//        telemetry.update();
    }


    private void setPowers(double power){
        BLMotor.setPower(power);
        BRMotor.setPower(power);
        FLMotor.setPower(power);
        FRMotor.setPower(power);
    }

    private void drive(double yIn, double xIn){
        double BLPower = yIn - xIn;
        double BRPower = yIn + xIn;
        double FLPower = yIn - xIn;
        double FRPower = yIn + xIn;

        double[] powers = {Math.abs(BLPower), Math.abs(BRPower), Math.abs(FLPower), Math.abs(FRPower)};
        Arrays.sort(powers);
        if (powers[3] > 1){
            BLPower /= powers[3];
            BRPower /= powers[3];
            FLPower /= powers[3];
            FRPower /= powers[3];
        }

        telemetry.addData("BLPower: ", BLPower);
        telemetry.addData("BRPower: ", BRPower);
        telemetry.addData("FLPower: ", FLPower);
        telemetry.addData("FRPower: ", FRPower);

        BLMotor.setPower(BLPower);
        BRMotor.setPower(BRPower);
        FLMotor.setPower(FLPower);
        FRMotor.setPower(FRPower);
    }
}
