/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.ImageSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoProperty;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;

import static frc.robot.Functions.*;
import static frc.robot.Const.*;

/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired from the USB camera, then a rectangle is put on the image
 * and sent to the dashboard. OpenCV has many methods for different types of
 * processing.
 */

public class Robot extends TimedRobot{
  Thread m_visionThread;

  @Override
  public void robotInit() {
    
    // VideoProperty chingchong = ImageSource.createProperty("minSolid", VideoProperty.Kind.kInteger, 14,18,1, 16,16);
    // ImageSource
    // CameraServerJNI.createSourceProperty(0, "fuck you this is a boolean", VideoProperty.Kind.kBoolean.getValue(), 0, 1, 1, 0, 0);

    m_visionThread = new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);
      camera.setFPS(60);
      camera.setExposureManual(2);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);
      CvSource outputStream2 = CameraServer.getInstance().putVideo("vl", 640, 480);

      Mat frame = new Mat();
      Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(9, 9));
      List<MatOfPoint> inputContours = new ArrayList<MatOfPoint>();
      MatOfPoint2f vertices = new MatOfPoint2f();
      
      Point[] all = new Point[8];
      double[][] coords = new double[8][2];
      double[][] top4 = new double[4][2];
      double[][] bot1 = new double[2][2];
      double[][] bot2 = new double[2][2];
      Point[] processed = new Point[8];
      double a1=0, b1=0, a2=0, b2=0, a3=0, b3=0;

      while (!Thread.interrupted()) {
        if (cvSink.grabFrame(frame) == 0) {
          outputStream.notifyError(cvSink.getError());
          continue;
        }

        // Tìm ra các thứ màu xanh (bao gồm cái phản quang)
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2HSV);
        Core.inRange(frame, lowerb, upperb, frame);
        Imgproc.dilate(frame, frame, kernel);
        Imgproc.erode(frame, frame, kernel);

        // Tìm outline của các phần liền nhau
        Imgproc.findContours(frame, inputContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Reset lại cái frame để vẽ vào sau
        frame = new Mat(frame.rows(), frame.cols(), CvType.CV_8U, Scalar.all(0));
        
        // Xem input trong stream Rectangle
        Imgproc.drawContours(frame, inputContours, -1, new Scalar(255, 255, 255));
        outputStream.putFrame(frame);

        // Reset tiếp
        frame = new Mat(frame.rows(), frame.cols(), CvType.CV_8U, Scalar.all(0));

        // Tìm cái phản quang
        vertices = findTarget(inputContours);

        // fewfijeofwef2ufo2fo1o2u1f
        // 2oiwj1o3fj2qo
        // COPY VAO DAY OK
        {
          all = vertices.toArray();
          for(int o = 0; o < 8; o++){
            coords[o][0] = all[o].x;
            coords[o][1] = all[o].y;
          }
          Arrays.sort(coords, (a, b) -> Double.compare(a[0], b[0]));
          for(int o = 0; o < 8; o++){
            if(o < 4){
              top4[o][0] = coords[o][0];
              top4[o][1] = coords[o][1];
            }else if(o < 6){
              bot1[o-4][0] = coords[o][0];
              bot1[o-4][1] = coords[o][1];
            }else{
              bot2[o-6][0] = coords[o][0];
              bot2[o-6][1] = coords[o][1];
            }
          }

          Arrays.sort(top4, (a, b) -> Double.compare(a[0], b[0]));
          a1 = top4[0][1] - top4[0][0] / top4[1][1] - top4[1][0];
          b1 = top4[0][1] - a1*top4[0][0];
          a2 = -1/a1;
          b2 = top4[2][1] - a2*top4[2][0];
          top4[2][0] = (b2-b1)/(a1-a2);
          top4[2][1] = a1*top4[2][0] + b1;
          a3 = -1/a1;
          b3 = top4[3][1] - a3*top4[3][0];
          top4[3][0] = (b3-b1)/(a1-a3);
          top4[3][1] = a3*top4[3][0] + b3;
          for(int o = 0; o < 4; o++){
            processed[o] = new Point(top4[o][0], top4[o][1]);
          }
        }
        
        // !!!
        // CHỌN CHỈ 1 TRONG 2 CÁI DƯỚI
        // !!!

        // 1) lệnh xử lý toàn bộ viền một contour
        // Imgproc.drawContours(frame, outputContours, -69420, new Scalar(255, 255, 255));
        
        // 2) nối các đỉnh/góc của contour
        MatOfPoint passoagresso = new MatOfPoint(processed);
        List<MatOfPoint> fuckyou = new ArrayList<MatOfPoint>();
        fuckyou.add(passoagresso);
        Imgproc.polylines(frame, fuckyou, true, iwanttodie);

        // ỉa Mat đã xử lý lên stream vl
        outputStream2.putFrame(frame);  

        // Xóa các cái contour cũ
        inputContours.clear();
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

}