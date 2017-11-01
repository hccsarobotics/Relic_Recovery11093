/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import java.lang.Math;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RecovererDrive", group="Recoverer")
public class RecovererDriveTest extends OpMode{

    /* Declare OpMode members. */
    HardwareRecoverer robot       = new HardwareRecoverer(); // use the class created to define a Pushbot's hardware

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }


    // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    int           armTarget = 0;
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    boolean armStateOpen = true;
    boolean hasExecuted = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        double right_side;
        double left_side;
        double turn;
        boolean slow;
        double slowDub;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left_side = -gamepad1.left_stick_y;
        right_side = gamepad1.right_stick_y;
        turn = gamepad1.right_stick_x;
        slow = gamepad1.right_bumper;

        slowDub = (slow) ? 6:1.75;

        if (Math.abs(turn) > 0.25)
        {
            robot.ldBack.setPower(-turn/slowDub);
            robot.ldFront.setPower(turn/slowDub);
            robot.rdBack.setPower(-turn/slowDub);
            robot.rdFront.setPower(turn/slowDub);
        }
        else
        {
            robot.ldBack.setPower(left_side/slowDub);
            robot.ldFront.setPower(left_side/slowDub);
            robot.rdFront.setPower(right_side/slowDub);
            robot.rdBack.setPower(right_side/slowDub);
        }

        if (gamepad1.b)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    });
        }



        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad2.right_bumper && hasExecuted) {
            if (armStateOpen) {
                armStateOpen = Boolean.FALSE;
                clawOffset += .5;

            }   else {
                armStateOpen = Boolean.TRUE;
                clawOffset -= 1;
            }
            hasExecuted = false;
        }


        // Move both servos to new position.  Assume servos are mirror image of each other.
        if (hasExecuted == false && gamepad2.right_bumper == false){
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);
            hasExecuted = true;
        }


        // Use gamepad buttons to move the arm up (Y) and down (A)

        if (gamepad2.y)
        {
            armTarget = armTarget + 40;
        }
        else if (gamepad2.a)
        {
            armTarget = armTarget - 40;
        }

        if (Math.abs(robot.arm.getCurrentPosition()) > Math.abs(armTarget) + 100 || Math.abs(robot.arm.getCurrentPosition()) < Math.abs(armTarget) - 100)
        {
            robot.arm.setTargetPosition(Math.abs(armTarget));
            if (Math.abs(robot.arm.getCurrentPosition()) > Math.abs(armTarget) && armTarget > 0)
            {
                robot.arm.setPower(-.25);
            }
            else if (Math.abs(robot.arm.getCurrentPosition()) < Math.abs(armTarget) && armTarget < 2020)
            {
                robot.arm.setPower(.25);
            }
        }
        else
        {
            robot.arm.setPower(0);
        }

        if (gamepad2.dpad_down)
        {
            robot.jewelArm.setPosition(.5);
        }
        else
        {
            robot.jewelArm.setPosition(1);
        }




        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("arm state", "%s", armStateOpen);
        telemetry.addData("Target Position", armTarget);
        telemetry.addData("Current Position", robot.arm.getCurrentPosition());



    /*
     * Code to run ONCE after the driver hits STOP
     */

    }



    @Override
    public void stop() {
    }
}