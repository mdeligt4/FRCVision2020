/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.vision.CameraSolver;
import frc.robot.math.*;

import static frc.robot.Functions.*;
import static frc.robot.Const.*;

public class Robot extends TimedRobot{
  Thread m_visionThread;

  @Override
  public void robotInit() {

    m_visionThread = new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);
      camera.setFPS(60);
      camera.setExposureManual(4);


      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);
      CvSource outputStream2 = CameraServer.getInstance().putVideo("vl", 640, 480);

      Mat frame = new Mat();
      Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(9, 9));
      List<MatOfPoint> inputContours = new ArrayList<MatOfPoint>();
      MatOfPoint2f vertices = new MatOfPoint2f();
      

      Point[] all = new Point[8];
      double[][] coords = new double[8][2];
      double[] topL = new double[2];
      double[] topR = new double[2];
      double[] botL = new double[2];
      double[] botR = new double[2];
      double[][] temp = new double[4][2];

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
        // frame = new Mat(frame.rows(), frame.cols(), CvType.CV_8U, Scalar.all(0));
        Core.inRange(frame, new Scalar(1,1,1), new Scalar(0,0,0), frame);
        
        // Xem input trong stream Rectangle
        Imgproc.drawContours(frame, inputContours, -1, new Scalar(255, 255, 255));
        outputStream.putFrame(frame);
        
        // Reset tiếp
        // frame = new Mat(frame.rows(), frame.cols(), CvType.CV_8U, Scalar.all(0));
        Core.inRange(frame, new Scalar(1,1,1), new Scalar(0,0,0), frame);
        // Tìm cái phản quang
        vertices = findTarget(inputContours);

        // Xử lý cố định điểm
        // Phân tách tọa độ 8 điểm tìm được ra array 2D
        if(vertices!=null){
          try{
            all = vertices.toArray();
            for(int o = 0; o < 8; o++){
              coords[o][0] = all[o].x;
              coords[o][1] = all[o].y;
            }
            // // Sắp xếp các cặp tọa độ theo chiều Y tăng dần
            
            Arrays.sort(coords, (double[] a, double[] b) -> Double.compare(a[1], b[1]));
            for(int o = 0; o < 4; o++){
              temp[o] = coords[o];
            }
            Arrays.sort(temp, (double[] a, double[] b) -> Double.compare(a[0], b[0]));
            System.out.println(Arrays.deepToString(temp));
            topL = temp[0];
            topR = temp[3];
            for(int o = 4; o < 8; o++){
              temp[o - 4] = coords[o];
            }
            Arrays.sort(temp, (double[] a, double[] b) -> Double.compare(a[0], b[0]));
            if(temp[0][1]>temp[1][1])botL = temp[0];
            else botL = temp[1];
            if(temp[2][1]>temp[3][1])botR = temp[2];
            else botR = temp[3];
            
            Imgproc.circle(frame, new Point(topL[0],topL[1]), 6, new Scalar(255,0,0));
            Imgproc.circle(frame, new Point(topR[0],topR[1]), 6, new Scalar(255,0,0));
            Imgproc.circle(frame, new Point(botL[0],botL[1]), 6, new Scalar(255,0,0));
            Imgproc.circle(frame, new Point(botR[0],botR[1]), 6, new Scalar(255,0,0));

            System.out.println("top left: "+ Arrays.toString(topL));
            System.out.println("top right: "+ Arrays.toString(topR));
            System.out.println("bot left: "+ Arrays.toString(botL));
            System.out.println("bot right: "+ Arrays.toString(botR));
            
            CameraSolver sol = new CameraSolver(new Coordinates3D(-8.66, -15, 0), new Coordinates3D(8.66, -15, 0),
                  new Coordinates3D(-17.32, 0, 0), new Coordinates3D(17.32, 0, 0));

            sol.solve(new Coordinates2D(botL[0], botL[1]), new Coordinates2D(botR[0], botR[1]),
                  new Coordinates2D(topL[0],topL[1]), new Coordinates2D(topR[0],topR[1]));
            // Nối 8 đỉnh đã xử lí
            MatOfPoint processedMat = new MatOfPoint(all); // Mat mới gồm 8 điểm đã xử lí trong processed[]
            List<MatOfPoint> processedList = new ArrayList<MatOfPoint>();
            processedList.add(processedMat);
            Imgproc.polylines(frame, processedList, true, white);
          }catch(Exception e){}
        }
        // // out Mat đã xử lý lên stream vl
        outputStream2.putFrame(frame);


        // Xóa các cái contour cũ
        inputContours.clear();
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

}