
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <Robot_Navi_Euro20.h>
#include <stmRos.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>
#include <std_msgs/Float32.h>
#include <iostream>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetLightProperties.h>
#include <eurobot2024/RobotCmd.h>
#include <eurobot2024/EncoderReadings.h>
#include <eurobot2024/Pose.h>
using namespace std;

void publishEncoderReadings();
void publishCurrentPose();
void callback(const std_msgs::String& command);
void commandCallback(const eurobot2024::RobotCmdRequest & req,eurobot2024::RobotCmdResponse & res);
void evitement(const std_msgs:: Bool& message);


std_msgs::Float32 l1_p;
std_msgs::Bool sending;
extern ros::Publisher tirettePub;
extern bool evitementFlag;
extern float l1,l2,l3,r1,r2,r3 ;
extern float x_obst,y_obst;
extern int PWM_L,PWM_R;
extern volatile float current_x,current_y,current_phi_deg,current_phi_rad;
extern volatile double right_speed, left_speed;
extern volatile float spacing_encoder;
extern volatile double right_encoder_speed,left_encoder_speed;
extern volatile float left_radius,right_radius,spacing_encoder,spacing_wheel;
extern volatile long total_right_count, total_left_count;
float test;
double v_rx ;
double omega_r ;
extern volatile float phi_init_deg;

float Vx;
float Vz;





ros::NodeHandle nh;

std_msgs::String str_msg;

eurobot2024::EncoderReadings enc_read;
eurobot2024::Pose current_pose;

ros::Publisher encoderReadings("EncoderReadings", &enc_read);
ros::Publisher pose("Pose", &current_pose);



ros::Subscriber<std_msgs::Bool> sub("evitementFlag", &evitement);

ros::ServiceServer<eurobot2024::RobotCmdRequest, eurobot2024::RobotCmdResponse> subCommandStm("/StmCommand", &commandCallback );

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(encoderReadings);
  nh.advertise(pose);
  nh.advertiseService(subCommandStm);
  nh.subscribe(sub);
}

void loop(void)
{
 // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	publishCurrentPose();
	/*if(evitementFlag)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	*/
}

void publishEncoderReadings(){
	enc_read.leftCount=total_left_count;
	enc_read.rightCount=total_right_count;
	encoderReadings.publish(&enc_read);
	nh.spinOnce();
	HAL_Delay(20);
}

void publishCurrentPose(){
	current_pose.x=current_x;
	current_pose.y=current_y;
	current_pose.phi_deg=current_phi_deg;
	pose.publish(&current_pose);
	nh.spinOnce();
	HAL_Delay(150);
}



void evitement(const std_msgs::Bool& message){
	evitementFlag = message.data;
}

void commandCallback(const eurobot2024::RobotCmdRequest & req,eurobot2024::RobotCmdResponse & res){
	/*move_distance(200, 200);
	HAL_Delay(200);
	move_distance(-200, 200);
	HAL_Delay(200);
	rotate(180, 300);
	HAL_Delay(200);
	rotate(-180, 300);
	*/
	if(!strcmp(req.command,"pwm")){
		 	 PWM_L=2000;
		     PWM_R=2000;

		     run_motors();
		     HAL_Delay(3000);
		     stop_motors();

		     res.success=true;
		     res.message ="Success!";

	}
	else if(!strcmp("test",req.command)){
			move_distance(500, 200);
			HAL_Delay(200);
			move_distance(-500, 200);
			HAL_Delay(200);
			rotate(90, 300);
			HAL_Delay(200);
			rotate(-90, 300);

			res.success=true;
			res.message="Success!";

	}
	else if(!strcmp("robotLocate",req.command)){
			Robot_Locate(req.x, req.y, req.speed);
			res.success=true;
			res.message="Success!";
	}
	else if(!strcmp("moveDistance",req.command)){
			move_distance(req.x, req.speed);
			res.success=true;
			res.message="Success!";
		}
	else if(!strcmp("rotate",req.command)){
				rotate(req.phi, req.speed);
				res.success=true;
				res.message="Success!";
		}
	else if(!strcmp("orientate",req.command)){
					orientate(req.phi, req.speed);
					res.success=true;
					res.message="Success!";
	}
	else if(!strcmp("encoders",req.command)){
						for(int i=0;i<200;i++){
							publishEncoderReadings();
						}
						res.success=true;
						res.message="Success!";
		}
	else if(!strcmp("setCoords",req.command)){
							current_x=req.x;
							current_y=req.y;
							current_phi_deg=req.phi;
							current_phi_rad = req.phi/180*M_PI;
							res.success=true;
							res.message="Success!";
		}
	else if(!strcmp("setRaduis",req.command)){
								right_radius=req.x;
								left_radius=req.y;
								res.success=true;
								res.message="Success!";
		}
	else {
		res.success=false;
		res.message ="FAILED!";
	}
}


/*void commandCallback(const reel_euro2021::jdRequest & req,reel_euro2021::jdResponse& res){

	if(!strcmp(req.function,"WheelSpacing")){
		spacing_wheel=req.vitesse;
		res.result ="Done Setting Wheel Spacing";

	}
	else if(!strcmp("EncoderSpacing",req.function)){
		spacing_encoder=req.x;
		res.success=true;
		res.result ="Done Setting Encoder Spacing";

	}
	else if(!strcmp("RightRadius",req.function)){
		right_radius=req.x;
		res.success=true;
		res.result ="Done Setting Right Radius";
	}
	else if(!strcmp("LeftRadius",req.function)){
		left_radius=req.x;
		res.success=true;
		res.result ="Done Setting Left Radius";
	}else if(!strcmp("xy",req.function)){
		if(req.x!=0) {current_x=req.x;}
		if(req.y!=0) {current_y=req.y;}
		current_phi_rad=req.phi;
		current_phi_deg= rad_to_deg(current_phi_rad);
		res.success=true;
		res.result ="Done Setting Coordinates";
	}else if(!strcmp("Go",req.function)){
		if(req.vitesse==0){
			Robot_Locate(0, 0, 0);//   Robot_Locate(req.duration.sec,req.duration.nsec,400);
		}else{
			float x=req.x;
			float y=req.y;
			Robot_Locate(x,y,req.vitesse);
		}
		res.success=true;
		res.result ="Done Reaching Goallll";
	}else if(!strcmp("Gobst",req.function)){
		if(req.vitesse!=0){
			float x=req.x;
			float y=req.y;
			Robot_Locateobst(x,y,req.vitesse);
		}
		res.success=true;
		res.result ="Done Reaching Goallll";
	}
	else if(!strcmp("Rotate",req.function)){

			rotate(req.phi, req.vitesse);
			res.success=true;
			res.result ="Done Rotatinggggg";
		}
	else if(!strcmp("Orientate",req.function)){
					float x=req.phi;
					orientate(x, req.vitesse);
					res.success=true;
					res.result ="Done Rotating";
				}
	else if(!strcmp("Move",req.function)){
						float x=req.x;
						move_distance(x, req.vitesse);
						res.success=true;
						res.result ="Done Moving";
					}

	else if(!strcmp("Asta3",req.function)){
					float x=req.x;
				asta3(x, req.vitesse);
				res.success=true;
				res.result ="Done Asta3";
			}
	else if(!strcmp("Asta3L",req.function)){
						float x=req.x;
					asta3L(x, req.vitesse);
					res.success=true;
					res.result ="Done Asta3 Left";
				}

	else if(!strcmp("Motors",req.function)){

						move_distance(200, 200);
						move_distance(-200, 200);
						orientate(90, 300);
						orientate(0, 300);
						res.success=true;
						res.result ="Done Testing Motors";
					}
	else if (!strcmp("GoCurv",req.function)){
		if(req.vitesse==0){
			Robot_locateCurv(0, 0, 0,0);//   Robot_Locate(req.duration.sec,req.duration.nsec,400);
				}else{
					float x=req.x;
					float y=req.y;
					float phi=req.phi;

					Robot_locateCurv(x,y,phi,req.vitesse);
				}
				res.success=true;
				res.result ="Done Reaching Goallll ye jabri";

	}
	else if (!strcmp("GoMultiCurv",req.function)){


						int n =req.nombre;
						reel_euro2021::c* m;
						m=req.curv_path;
						float** mat=(float**)malloc(n*sizeof(float*));
						for (int index=0;index<n;++index)
						{
						    mat[index] = (float*)malloc(3 * sizeof(float));
						}

						for (int i=0;i<n;i++){
							mat[i][0]=m[i].x;
							mat[i][1]=m[i].y;
							mat[i][2]=m[i].phi;
						}
						Robot_Locate_Multi_Curv(mat,n,req.vitesse);

					res.success=true;
					res.result ="Done Reaching Goallll ye jabri";

		}
	else{
		res.success=false;
				res.result ="FAILED !!!!!!!!!";
	}


}

*/

void rotateAck(){
	//sending.data=true;
	//sending_pub.publish(&sending);
}
