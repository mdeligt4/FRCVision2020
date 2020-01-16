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

import com.sun.source.tree.Tree.Kind;

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
    // double H = 61/2, tH = 20;
    // double S = 11.8*2.55, tS = 15;
    // double V = 92.5*2.55, tV = 15;
    // Scalar lowerb = new Scalar(H-tH,S-tS,V-tV);
    // Scalar upperb = new Scalar(H+tH,S+tS,V+tV);
    double lH = 50 / 2;
    double lS = 30 * 2.55;
    double lV = 25 * 2.55;
    double hH = 110 / 2;
    double hS = 70 * 2.55;
    double hV = 70 * 2.55;
    Scalar lowerb = new Scalar(lH, lS, lV);
    Scalar upperb = new Scalar(hH, hS, hV);
    Scalar iwanttodie = new Scalar(255, 255, 255);
    double minSolidity = 0.15, maxSolidity = 0.2;
    double minAspectRatio = 0.020, maxAspectRatio = 0.028;

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
      List<MatOfPoint> outputContours = new ArrayList<MatOfPoint>();
      MatOfPoint contour = new MatOfPoint();

      MatOfPoint2f vertices = new MatOfPoint2f();

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
        Core.inRange(frame, new Scalar(1, 1, 1), new Scalar(0, 0, 0), frame);

        // Imgproc.convexHull(contours.get(0), hulls.get(0));

        Imgproc.drawContours(frame, inputContours, -1, new Scalar(255, 255, 255));
        // Imgproc.drawContours(frame, hulls, 0, new Scalar(255,255,255));

        // ImageSource.createProperty();
        
        for(int i=0;i<inputContours.size();i++){
          contour = inputContours.get(i);

          // Xử lí các đỉnh
          vertices.fromList(contour.toList());
          Imgproc.approxPolyDP(vertices, vertices, Imgproc.arcLength(vertices, true) * 0.01, true);
          if(vertices.total()<7||vertices.total()>9)continue;

          // Tỉ lệ chiều cao / chiều dài
          double aspectRatio = getAspectRatio(contour);
          System.out.print(Imgproc.contourArea(contour) + " " +  aspectRatio + " ");

          // Tỉ lệ diện tích contour / convex hull
          double solidity = getSolidity(contour);
          System.out.println(solidity);

          if(solidity<minSolidity||solidity>maxSolidity)continue;
          if(aspectRatio<minAspectRatio||aspectRatio>maxAspectRatio)continue;

          outputContours.add(contour);
        }

        outputStream.putFrame(frame);

        Core.inRange(frame, new Scalar(1, 1, 1), new Scalar(0, 0, 0), frame);
        
        // !!!
        // CHỌN CHỈ 1 TRONG 2 CÁI DƯỚI
        // !!!

        // 1) lệnh xử lý toàn bộ viền một contour
        // Imgproc.drawContours(frame, outputContours, -69420, new Scalar(255, 255, 255));
        
        // 2) nối các đỉnh/góc của contour
        MatOfPoint passoagresso = new MatOfPoint(vertices.toArray());
        List<MatOfPoint> fuckyou = new ArrayList();
        fuckyou.add(passoagresso);
        Imgproc.polylines(frame, fuckyou, true, iwanttodie);

        // ỉa Mat đã xử lý lên stream vl
        outputStream2.putFrame(frame);  

        // Xóa các cái contour cũ
        inputContours.clear();
        outputContours.clear();
        
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }



  MatOfPoint getHull(MatOfPoint contour){
    MatOfInt h = new MatOfInt();
    MatOfPoint hull = new MatOfPoint();
    Imgproc.convexHull(contour, h);
    hull.create((int)h.size().height,1,CvType.CV_32SC2);
    for(int j=0;j<h.size().height;j++){
      int index = (int) h.get(j,0)[0];
      double[] point = new double[]{contour.get(index,0)[0],contour.get(index,0)[1]};
      hull.put(j,0,point);
    }
    return hull;
  }

  double getSolidity(MatOfPoint contour){
    double hull = Imgproc.contourArea(getHull(contour));
    if(hull==0)return -1;
    return Imgproc.contourArea(contour)/hull;
  }

  double getAspectRatio(MatOfPoint contour){
    double sum = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true)/2;
    double product = Imgproc.contourArea(contour);
    double x,y;
    double delta = Math.pow(sum, 2)-4*product;
    if(delta<0)return -1;
    x=(sum-Math.sqrt(delta))/2;
    y=(sum+Math.sqrt(delta))/2;
    if(y==0)return -1;
    return x/y;
  }
}