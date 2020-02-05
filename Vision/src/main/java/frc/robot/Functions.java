package frc.robot;

import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;

import static frc.robot.Const.*;
public class Functions {
  public static MatOfPoint getHull(MatOfPoint contour){
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

  public static double getSolidity(MatOfPoint contour){
    double hull = Imgproc.contourArea(getHull(contour));
    if(hull==0)return -1;
    return Imgproc.contourArea(contour)/hull;
  }
 
  public static double getAspectRatio(MatOfPoint contour){
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
  
  public static MatOfPoint2f findTarget(List<MatOfPoint> contours){
    MatOfPoint contour;
    MatOfPoint2f vertices = new MatOfPoint2f();
    double maxArea=-1;
    MatOfPoint2f result=null;
    for(int i=0;i<contours.size();i++){
      contour = contours.get(i);

      // Diện tích bé quá thì bye
      if(Imgproc.contourArea(contour)<maxArea)continue;

      // Phải có 8 đỉnh
      vertices.fromList(contour.toList());
      Imgproc.approxPolyDP(vertices, vertices, Imgproc.arcLength(vertices, true) * 0.005, true);
      if(vertices.total()!=8)continue;


      // // Tỉ lệ chiều cao / chiều dài
      // double aspectRatio = getAspectRatio(contour);
      // System.out.print(Imgproc.contourArea(contour) + " " +  aspectRatio + " ");

      // // Tỉ lệ diện tích contour / convex hull
      // double solidity = getSolidity(contour);
      // System.out.println(solidity);

      // if(solidity<minSolidity||solidity>maxSolidity)continue;
      // if(aspectRatio<minAspectRatio||aspectRatio>maxAspectRatio)continue;

      maxArea=Imgproc.contourArea(contour);
      result = vertices;
    }
    return result;
  }
}
