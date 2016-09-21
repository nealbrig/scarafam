package main;


/**
 * Class represents SCARA robotic arm.
 *
 * @Arthur Roberts
 * @0.0
 */

import ecs100.UI;
import java.awt.Color;
import java.util.*;

public class Arm
{

	private static boolean isDrawing = false;

    // fixed arm parameters
    private int xm1;  // coordinates of the motor(measured in pixels of the picture)
    private int ym1;
    private int xm2;
    private int ym2;
    private double r;  // length of the upper/fore arm
    // parameters of servo motors - linear function pwm(angle)
    // each of two motors has unique function which should be measured
    // linear function cam be described by two points
    // motor 1, point1
    private double pwm1_val_1;
    private double theta1_val_1;
    // motor 1, point 2
    private double pwm1_val_2;
    private double theta1_val_2;

    // motor 2, point 1
    private double pwm2_val_1;
    private double theta2_val_1;
    // motor 2, point 2
    private double pwm2_val_2;
    private double theta2_val_2;


    // current state of the arm
    private double theta1; // angle of the upper arm
    private double theta2;

    private double xj1;     // positions of the joints
    private double yj1;
    private double xj2;
    private double yj2;
    private double xt;     // position of the tool
    private double yt;
    private boolean valid_state; // is state of the arm physically possible?

    /**
     * Constructor for objects of class Arm
     */
    public Arm()
    {
        xm1 = 290; // set motor coordinates
        ym1 = 372;
        xm2 = 379;
        ym2 = 374;
        r = 156.0;
        theta1 = -90.0*Math.PI/180.0; // initial angles of the upper arms
        theta2 = -90.0*Math.PI/180.0;
        valid_state = false;
    }

    // draws arm on the canvas
    public void draw()
    {
    	if (Arm.isDrawing){
    		System.out.println("protected from mutli threading");
    		return;
    	}
    	Arm.isDrawing = true;
        // draw arm
        int height = UI.getCanvasHeight();
        int width = UI.getCanvasWidth();
        // calculate joint positions
        xj1 = xm1 + r*Math.cos(theta1);
        yj1 = ym1 + r*Math.sin(theta1);
        xj2 = xm2 + r*Math.cos(theta2);
        yj2 = ym2 + r*Math.sin(theta2);

        //draw motors and write angles
        int mr = 20;
        UI.setLineWidth(5);
        UI.setColor(Color.BLUE);
        UI.drawOval(xm1-mr/2,ym1-mr/2,mr,mr);
        UI.drawOval(xm2-mr/2,ym2-mr/2,mr,mr);
        // write parameters of first motor
        String out_str=String.format("t1=%3.1f",theta1/**180/Math.PI*/);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+2*mr);
        out_str=String.format("xm1=%d",xm1);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+3*mr);
        out_str=String.format("ym1=%d",ym1);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+4*mr);
        // ditto for second motor
        out_str = String.format("t2=%3.1f",theta2/**180/Math.PI*/);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+2*mr);
        out_str=String.format("xm2=%d",xm2);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+3*mr);
        out_str=String.format("ym2=%d",ym2);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+4*mr);
        // draw Field Of View
        UI.setColor(Color.GRAY);
        UI.drawRect(0,0,640,480);

       // it can b euncommented later when
       // kinematic equations are derived
        if ( valid_state) {
          // draw upper arms
          UI.setColor(Color.GREEN);
          UI.drawLine(xm1,ym1,xj1,yj1);
          UI.drawLine(xm2,ym2,xj2,yj2);
          //draw forearms
          UI.drawLine(xj1,yj1,xt,yt);
          UI.drawLine(xj2,yj2,xt,yt);
          // draw tool
          double rt = 20;
          UI.drawOval(xt-rt/2,yt-rt/2,rt,rt);
        }
        UI.setColor(Color.BLACK);
        UI.setLineWidth(1);

        UI.drawLine(this.xj1, this.yj1, this.xj2, this.yj2);

        UI.drawOval(this.xj1 - this.r, this.yj1 - this.r, this.r*2, this.r*2);
        UI.drawOval(this.xj2 - this.r, this.yj2 - this.r, this.r*2, this.r*2);

        UI.setLineWidth(5);

        Arm.isDrawing = false;
   }

   // calculate tool position from motor angles
   // updates variable in the class
   public void directKinematic(){

       // midpoint between joints
       //double  xa =.... ;
       //double  ya =.... ;
       // distance between joints
       //double d = ...;
       double d=0;//@TODO change
       if (d<2*r){
           valid_state = true;
         // half distance between tool positions
         //double  h = ...;
         //double alpha= ...;
         // tool position
        // double xt = ...;
        // double yt = ...;
         //  xt2 = xa - h.*cos(alpha-pi/2);
         //  yt2 = ya - h.*sin(alpha-pi/2);
       } else {
           valid_state = false;
        }

    }

    // motor angles from tool position
    // updetes variables of the class
    public void inverseKinematic(double xt_new,double yt_new){

        valid_state = true;
        xt = xt_new;
        yt = yt_new;

        double dx1 = Math.abs(xt - xm1);
        double dy1 = Math.abs(yt - ym1);

        double dx2 = Math.abs(xt - xm2);
        double dy2 = Math.abs(yt - ym2);

        double d_from_m1 = Math.sqrt(dx1*dx1+dy1*dy1);
        double d_from_m2 = Math.sqrt(dx2*dx2+dy2*dy2);

        if(d_from_m1 < r || d_from_m2 < r || d_from_m1 > 2*r || d_from_m2 > 2*r){
            valid_state = false;
            return;
        }

        double theta1_p1 = Math.atan(dy1/dx1);
        double theta2_p1 = Math.atan(dy2/dx2);

        if (xt < xm1)theta1_p1 = Math.atan(dx1/dy1) + Math.PI/2;
        if (xt > xm2)theta2_p1 = Math.atan(dx2/dy2) + Math.PI/2;

        double theta1_p2 = Math.acos((d_from_m1/2)/r);
        double theta2_p2 = Math.acos((d_from_m2/2)/r);

        this.theta1 = -(theta1_p1 + theta1_p2);
        this.theta2 = -(Math.PI - (theta2_p1 + theta2_p2));

        return;
    }

    // returns angle of motor 1
    public double get_theta1(){
        return theta1;
    }
    // returns angle of motor 2
    public double get_theta2(){
        return theta2;
    }
    // sets angle of the motors
    public void set_angles(double t1, double t2){
        theta1 = t1;
        theta2 = t2;
    }

    // returns motor control signal
    // for motor to be in position(angle) theta1
    // linear intepolation
    public int get_pwm1(){
        int pwm = (int) ((Math.abs(theta1) + 13.399)/0.0957);
        return pwm;
    }
    // ditto for motor 2
    public int get_pwm2(){
        int pwm = (int) ((Math.abs(theta2) + 73.193)/0.1014);
        //pwm = (int)(pwm2_90 + (theta2 - 90)*pwm2_slope);
        return pwm;
    }

 }
