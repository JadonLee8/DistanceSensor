package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@TeleOp
public class RumbleInRange extends OpMode {

    private ColorSensor cSensor;
    private DistanceSensor sensor;
    private DcMotorEx BLMotor;
    private DcMotorEx BRMotor;
    private DcMotorEx FLMotor;
    private DcMotorEx FRMotor;

    @Override
    public void init() {
        cSensor = hardwareMap.get(ColorSensor.class, "cSensor");
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
        drive(gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (sensor.getDistance(DistanceUnit.INCH) - 2 < 2 && sensor.getDistance(DistanceUnit.INCH) - 2 > 1){
            gamepad1.rumble(100);
        }

        telemetry.addData("sensor reading: ", sensor.getDistance(DistanceUnit.INCH) - 2);
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

        BLMotor.setPower(0.65 * BLPower);
        BRMotor.setPower(0.65 * BRPower);
        FLMotor.setPower(0.65 * FLPower);
        FRMotor.setPower(0.65 * FRPower);
    }
}
