/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "Object Detection Webcam Functions", group = "Concept")
//@Disabled
public class ExampleObjectDetectionWebcam3 extends LinearOpMode {
  /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
   * the following 4 detectable objects
   *  0: Ball,
   *  1: Cube,
   *  2: Duck,
   *  3: Marker (duck location tape marker)
   *
   *  Two additional model assets are available which only contain a subset of the objects:
   *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
   *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
   */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {"Ball","Cube","Duck","Marker"};
    String DuckPosition;
    float center;

    private static final String VUFORIA_KEY =
            "AVuwooL/////AAABmVcNitcih0Fkhok8emRlRE1As/ewLXPEdUwV+ijYqYxoVRP+agLwn1kFcLodqjKMD3EXLlv6BKliVOYVVDJz3lIysaHnRsJ9CT9sIZZwQBJ2tmqLP52VM8qB0Kq7mhs+Qu7J5fz6ritQhbSEiFziJ/teJ+N2XkkvSNXTtwMfdDgXLJ1xaxjEmX+r0o4dJWcpU6ar4JX+S2U0LO3JkmQjxXXOOFBGtJiH8t1yw2t3QqJT5AjJxHhkcBlWOU6AQqHdKxiyRugdQa6fHWb9i0rCiv22lwTELzRxfZNckR+iZKTpngqHNzMpfZloGO8rdOe5IAcAe5BA3HU0nUMzglzgdPwWCEImFVSShV7lxhIVRa2c";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {


        initDetectObjectPosition();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        DuckPosition = detectObjectPosition();

        if (DuckPosition == "Left"){
            telemetry.addLine("run left auto");


        }else if(DuckPosition == "Right"){
            RedLeftAuto1();
            telemetry.addLine("run right auto");
        }else if(DuckPosition == "Middle"){
            telemetry.addLine("run middle auto");
        } else{
            telemetry.addLine("Detection did not work");
        }

            while (opModeIsActive()){
                telemetry.update();
            }

    }


    private void RedLeftAuto1(){



    }
    private void initDetectObjectPosition(){
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.5, 16.0/9.0); //2.5
        }

    }

    private String detectObjectPosition(){
        String position =" ";
        String object =" ";

        if (opModeIsActive()) {
            // while (opModeIsActive()) { // You don't want to have this part in a loop, it should run only once. ASAPH
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    for (Recognition recognition : updatedRecognitions) {

                        object = recognition.getLabel(); // Used a variable to store the object detected. ASAPH

                        if(object == "Duck"){
                        center = recognition.getLeft() + ((recognition.getRight() - recognition.getLeft())/2); // height of image is 480 and width is 640 ASAPH
                    }
                    }

                    telemetry.update();
                }
            }

        }

        if (tfod != null) {
            tfod.shutdown(); // This will stop the camera. ASAPH
        }

        if (center < 200) {
            position = "Left";
        }
        else if(center > 400){
            position = "Right";
        }
        else if(center > 200 && center < 400){
            position = "Middle";
        }
        else{
            position = " ";
        }

        //while(opModeIsActive()){
            telemetry.addData("center", center);
            //telemetry.update();
       // }

        return position;
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
