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

package org.firstinspires.ftc.teamcode.drive;

import android.graphics.Color;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;


/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Drive", group="Iterative OpMode")
//@Disabled
public class Drive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private  DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private  DcMotor launchLeft = null;
    private  DcMotor launchRight = null;
    private  DcMotor revolver = null;
    private  DcMotor intake = null;
    private Servo wrist = null;
    private Servo intakeArm=null;
    boolean prevA = false;
    boolean prevY = false;
    boolean prevX = false;
    boolean prevB = false;
    boolean noprevB= false;
    //revolver variables
    double Pos1=0;
    double Pos2=112;
    double Pos3=235;
    double tolerance=6.0;
    float gain = 2;
    double TPR = 384.5; //Ticks per rot


    /* The colorSensor field will contain a reference to our color sensor hardware object */
    NormalizedColorSensor colorSensor;

    private HuskyLens huskyLens;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right drive");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left front drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right front drive");
        launchLeft = hardwareMap.get(DcMotor.class, "launchleft");
        launchRight = hardwareMap.get(DcMotor.class, "launchright");
        intake = hardwareMap.get(DcMotor.class, "intake");
        revolver =hardwareMap.get(DcMotor.class, "revolver");
        wrist =hardwareMap.get(Servo.class, "wrist");
        intakeArm =hardwareMap.get(Servo.class, "intakeArm");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        revolver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    //int servo


    @Override
    public void loop() {
        // Setup variables
        double frontLeftPower;
        double leftPower;
        double frontRightPower;
        double rightPower;
        double strafe;
        double shoot;
        double Intake;
        double Revolver;

        final float[] hsvValues = new float[3];

        //colors



        int revolverPos =revolver.getCurrentPosition();
        // convert TPR to degrees
        double revolverCurentPos = revolverPos % TPR;
       if (revolverCurentPos < 0) revolverCurentPos += TPR; //fixes negative values

        //convert to angle
    double angle =(revolverCurentPos / TPR) *360.0;




        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
         strafe = gamepad1.left_stick_x;
        frontLeftPower    = Range.clip(drive *.9 + turn*.9 - strafe*.9, -1.0, 1.0) ;
        leftPower    = Range.clip(drive*.9 + turn*.9 + strafe*.9, -1.0, 1.0) ;
        rightPower   = Range.clip(drive*.9 - turn*.9 - strafe*.9, -1.0, 1.0) ;
        frontRightPower   = Range.clip(drive*.9 - turn*.9 + strafe*.9, -1.0, 1.0) ;

        //shoot the artifacts
        if(gamepad2.a)
            shoot=-.75;
        else
            shoot=0;

        //intake
        if(gamepad2.y)
            Intake=-1;
        else
            Intake=0;
        //revolver colorSensor logic

        //revolver

        //revolver pos selection
        if (gamepad2.left_bumper)
            Revolver=-0.1;
        else if (gamepad2.right_bumper) {
            Revolver=.1;
        }
        else Revolver=0;


        //if the revolver is in the right place then the servo will activate
       /* if (gamepad2.x &&
            (Math.abs(angle - Pos1) < tolerance ||
            Math.abs(angle - Pos2) < tolerance ||
            Math.abs(angle - Pos3) <tolerance))
        intakeArm.setPosition(.4);
        else
            intakeArm.setPosition(0);
*/
        if (gamepad2.x)
            intakeArm.setPosition(.4);
        else
            intakeArm.setPosition(0);
        //toggle logic
        if(gamepad2.b && !prevB) {
            noprevB = !noprevB;
            ;
        }
        prevB=gamepad2.b;

        //launch angle
        if(noprevB)
            wrist.setPosition(1);
        else
            wrist.setPosition(.55);

        if(gamepad2.right_bumper);








        //toggle buttons
        prevY = gamepad2.y;
        prevA = gamepad2.a;
        prevX = gamepad2.x;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        //send power to other motors
        launchLeft.setPower(shoot);
        launchRight.setPower(-shoot);
        intake.setPower(Intake);
       revolver.setPower(Revolver);

        //change gain of color

        telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
        telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

        // Update the gain value if either of the A or B gamepad buttons is being held
        if (gamepad1.a) {
            // Only increase the gain by a small amount, since this loop will occur multiple times per second.
            gain += 0.005;
        } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
            gain -= 0.005;
        }

        //detect color
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);

        // Show the gain value via telemetry
        telemetry.addData("Gain", gain);

        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensor.setGain(gain);





        //show pos of revolver on Driver Hub
    telemetry.addData("revolverPos",angle);
    //show wrist pos
        telemetry.addData("wristPos", wrist.getPosition());
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
