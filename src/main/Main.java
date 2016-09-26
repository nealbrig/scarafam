package main;

/* Code for Assignment ??
 * Name:
 * Usercode:
 * ID:
 */


import ecs100.*;

import java.awt.*;


/** <description of class Main>
 */
public class Main{

	private double pixel_size = 5.7017;

	private Arm arm;
	private Drawing drawing;
	private ToolPath tool_path;
	// state of the GUI
	private int state; // 0 - nothing
	// 1 - inverse point kinematics - point
	// 2 - enter path. Each click adds point
	// 3 - enter path pause. Click does not add the point to the path

	/**      */
	
	/*#
		Adding the below variables for circle drawing:
	*/
	
	private int numCirclePoints = 20;
	private boolean doCircle = false;
	
	
	public Main(){
		UI.initialise();
		UI.addButton("xy to angles", this::inverse);
		UI.addButton("Enter path XY", this::enter_path_xy);
		UI.addButton("Save path XY", this::save_xy);
		UI.addButton("Load path XY", this::load_xy);
		UI.addButton("Save path Ang", this::save_ang);
		UI.addButton("Load path Ang:Play", this::load_ang);
		UI.addButton("save PWM", this::save_pwm);
		
		//Added another button below:
		
		UI.addButton("Draw Circle - Use with Enter path XY", () -> this.doCircle = true);

		// UI.addButton("Quit", UI::quit);
		UI.setMouseMotionListener(this::doMouse);
		UI.setKeyListener(this::doKeys);


		//ServerSocket serverSocket = new ServerSocket(22);

		this.arm = new Arm();
		this.drawing = new Drawing();
		this.tool_path = new ToolPath();
		this.run();
		arm.draw();
	}

	public void save_pwm(){
		String fname = UIFileChooser.open();
		tool_path.convert_drawing_to_angles(drawing,arm,fname);
		tool_path.convert_angles_to_pwm(arm);
		tool_path.save_pwm_file(fname);
	}

	public void doKeys(String action){
		UI.printf("Key :%s \n", action);
		if (action.equals("b")) {
			// break - stop entering the lines
			state = 3;
			//

		}

	}


	public void doMouse(String action, double x, double y) {
		//UI.printf("Mouse Click:%s, state:%d  x:%3.1f  y:%3.1f\n",
		//   action,state,x,y);
		UI.clearGraphics();
		String out_str=String.format("%3.1f %3.1f",x,y);
		UI.drawString(out_str, x+10,y+10);
		//
		
		/*#
			Adding a new if statement to handle the drawing of the perfect circle
		*/
		if(doCircle && action.equals("clicked")){
			doCircle = false;
			state = 2;
			double xCoord, yCoord;
			//circle diameter must be 50mm
			double rad = 25/pixel_size; // radius of circle
			
			double theta = 0; // angle of point with respect to the circle centre
			// the x and y coordinates at the time of the click will be the centre point coordinates
			
			double stepValue = 360/numCirclePoints; // the increment for each loop in the while loop
			
			while(theta <= 360){ // note that if numCirclePoints is say 10, then 11 points will be made to ensure that the circle is connected back to the starting point
				xCoord = x + rad*Math.cos(Math.toRadians(theta));
				yCoord = y + rad*Math.sin(Math.toRadians(theta));
				doMouse("clicked", xCoord, yCoord); // Taking a slightly recursive approach to adding the points autonomously
				theta += stepValue;
			}
			return; // nessecary otherwise an extra point will be added at the centre of the circle
			
		}
		
		
		
		if ((state == 1)&&(action.equals("clicked"))){
			// draw as

			arm.inverseKinematic(x,y);
			arm.draw();
			return;
		}

		if ( ((state == 2)||(state == 3))&&action.equals("moved") ){
			// draw arm and path

			arm.inverseKinematic(x,y);
			arm.draw();

			// draw segment from last entered point to current mouse position
			if ((state == 2)&&(drawing.get_path_size()>0)){
				PointXY lp = new PointXY();
				lp = drawing.get_path_last_point();
				//if (lp.get_pen()){
				UI.setColor(Color.GRAY);
				UI.drawLine(lp.get_x(),lp.get_y(),x,y);
				// }
			}
			drawing.draw();
		}

		// add point
		if (   (state == 2) &&(action.equals("clicked"))){
			// add point(pen down) and draw
			UI.printf("trying to add point x=%f y=%f\n",x,y);
			// add point with pen down

			arm.inverseKinematic(x,y);
			if(arm.is_valid_state()){
				drawing.add_point_to_path(x,y,true);
				arm.draw();
				drawing.draw();
				drawing.print_path();
			}
		}


		if (   (state == 3) &&(action.equals("clicked"))){
			// add point and draw
			//UI.printf("Adding point x=%f y=%f\n",x,y);


			arm.inverseKinematic(x,y);
			if(arm.is_valid_state()){
				drawing.add_point_to_path(x,y,false); // add point with pen up
				arm.draw();
				drawing.draw();
				drawing.print_path();
				state = 2;
			}
		}
	}


	public void save_xy(){
		state = 0;
		String fname = UIFileChooser.save();
		drawing.save_path(fname);
	}

	public void enter_path_xy(){
		state = 2;
	}

	public void inverse(){
		state = 1;
		arm.draw();
	}

	public void load_xy(){
		state = 0;
		String fname = UIFileChooser.open();
		drawing.load_path(fname);
		drawing.draw();

		arm.draw();
	}

	// save angles into the file
	public void save_ang(){
		String fname = UIFileChooser.open();
		tool_path.convert_drawing_to_angles(drawing,arm,fname);
		tool_path.save_angles(fname);
	}


	public void load_ang(){

	}

	public void run() {
		while(true) {
			arm.draw();
			UI.sleep(20);
		}
	}

	public static void main(String[] args){
		System.out.println("Started");
		Main obj = new Main();
	}

}
