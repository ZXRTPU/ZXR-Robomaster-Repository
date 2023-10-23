#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "Can_user.h"
#include "Friction_task.h"
#include "Exchange_task.h"
#include "struct_typedef.h"

fp32 target_angle1;
uint8_t r_model= 0; //�жϵ��ָ�ģʽ 0��ֹ 1�� 2�ر�
uint8_t friction_flag = 0;

fp32 count = 0;
fp32 heat_warning = 0;
fp32 shoot_speed_set;
fp32 speed_choice;

//======================================�¶���ı���======================================
uint16_t Encoder_Num_Per_Round = 8192;  //������һȦ�ı���ֵ
#define Gearbox 36.0f; //������ٱ�

uint16_t Pre_encoder=0;
int32_t Total_Round=0; //�������ת��Ȧ��
int32_t Total_encoder=0;  //����������ۼ�ֵ
float Now_angle=0.0f;   //�ɱ�����ӳ����ĽǶ�ֵ

float target_angle_trigger=0;
float target_omega_trigger=0;

//======================================�Ƕ�ӳ�亯��-�Ƕȸ���==================================1
void angle_change()
{
    int16_t delta_encoder;
    //int16_t Pre_encoder=motor_info[2].rotor_angle;

    delta_encoder=motor_info[2].rotor_angle-Pre_encoder;
    Pre_encoder=motor_info[2].rotor_angle;

    if( delta_encoder<-4096)
    {
        //������ת��һȦ
        Total_Round++;
    }
    else if( delta_encoder>4096)
    {
        //������ת��һȦ
        Total_Round--;
    }

    Total_encoder=Total_Round*Encoder_Num_Per_Round+motor_info[2].rotor_angle;

    Now_angle=(float)Total_encoder/(float)Encoder_Num_Per_Round*2.0f*PI/Gearbox;
}


double msp(double x,double in_min,double in_max,double out_min,double out_max)
{
	
	return (x-in_min)*(out_max-out_min)/(in_max-out_max)+out_min;
}


//======================================�ٶȻ��ͽǶȻ����Ƴ���===============================
void angle_control()
{
    target_angle_trigger=PI/6;
	  //motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
	  Now_angle=msp(motor_info[2].rotor_angle,0,8191,-PI,PI);
   
	  target_omega_trigger=pid_calc(&motor_pid[2],target_angle_trigger,Now_angle);

	  motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_omega_trigger, motor_info[2].rotor_speed);
}

//======================================˫������PID
short Pid_Cascade_Cal(void)
{
    double target_angle_trigger= - Encoder_Num_Per_Round/18;
	  double target_omega_trigger=0;
	  
	  target_omega_trigger=(pid_pitch_calc(&motor_pid[2],target_angle_trigger,motor_info[2].rotor_angle))
	                         /(Encoder_Num_Per_Round*36);
	
	  motor_info[2].set_voltage=pid_pitch_calc(&motor_pid[2],target_omega_trigger, motor_info[2].rotor_speed);
}

//PID��ʼ��
static void Friction_init();

//ģʽѡ��
static void model_choice();

//����
static void magazine_task();
static void magazine_init();

//ǹ����������
static void heat_limit();

//can����
static void shoot_data_send();

//���⼤��ʹ��
static void laser_init();

static void friction_enable();
static void friction_disable();

static void friction_flag_change();


//����
static void trigger_task();


//���ݲ���ϵͳ�ı䵯��
static void shoot_speed_choice();


void Friction_task(void const * argument)  //��freeRTOS�����е����߳���
{
  Friction_init();
  magazine_init();
  laser_init();
		
  for(;;)
  {	  
	 shoot_speed_choice(); //�ٶȳ�ֵ����
		
	 model_choice(); //ģʽѡ��˳����Ħ���ֺͲ�צ��ʹ�俪ʼת����
		
	 magazine_task();
	 heat_limit();       
	
   shoot_data_send();	//�������pid�����õ���ֵͨ��canͨ�ŷ��͸����
  }
  osDelay(1);
}


static void Friction_init()
{
  pid_init(&motor_pid[0], 30,   0, 0, 16384, 16384); 
	pid_init(&motor_pid[1], 30,   0, 0, 16384, 16384);
	pid_init(&motor_pid[2], 30 ,  0, 0,10000 , 10000);	
}

static void laser_init()
{
	HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_SET);
}


//===========================================ǹ�ڵ���ѡ��===========================================
static void shoot_speed_choice()
{
	
		if(infantry.speed_limit == 15 ) //��������
		{
			shoot_speed_set = 3950;
		}
		else if(infantry.speed_limit == 18 ) //��ȴ����
		{
			shoot_speed_set = 4625;  
		}
		else if(infantry.speed_limit == 30) //��������
		{
			shoot_speed_set = 10000;
		}
		else
		{
			shoot_speed_set = 8700;
		}
	
}


//===========================================����Ħ����============================================================
static void model_choice()
{
	friction_flag_change(); //��ȡ�����Ƿ���
	if(friction_flag||rc_ctrl.rc.s[1] == 1)
	{
		friction_enable();
	}
	else //����������ת����0
	{
		friction_disable();
	}
	
	trigger_task();
}


static void friction_enable()
{
     //�ڴ˴����ٶ������޸�   
	
		target_speed[0] = -shoot_speed_set;//-8700
    target_speed[1] = shoot_speed_set;// 8700
	
		motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
    motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
	
}


static void friction_disable()
{
	
	  target_speed[0] = 0;
    target_speed[1] = 0;
	  
	  motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
    motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
	
}


static void friction_flag_change()
{
	if(q_flag)
	{
		friction_flag = friction_flag + 1;
	}
	else if(e_flag)
	{
		friction_flag = 0;
	}
}


//=============================================���̵��==========================================================
//=====================================�Բ��̵�����ƴ��������޸�
static void trigger_task()
{
	if(rc_ctrl.mouse.press_l || rc_ctrl.rc.s[0] == 1)
	{
		target_speed[2] = -500; //-1500;   //�ı䲦��ת������ʱһ��ֱ�ӵ���Ŀ���ٶ�
		motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
	}
	else if( rc_ctrl.rc.s[0] == 2)
	{
		angle_control();
		/*
		target_angle_trigger=30;
		target_omega_trigger=pid_pitch_calc(&motor_pid[2],target_angle_trigger,motor_info[2].rotor_angle);
		motor_info[2].set_voltage = pid_calc(&motor_pid[2],target_omega_trigger, motor_info[2].rotor_speed);*/
	  /*Pid_Cascade_Cal();*/
	}
	else
	{
		target_speed[2] = 0;
	 	motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
	}
}


//=============================================���ָ�==========================================================
static void magazine_task()
{

	if(r_flag)
	{
		target_speed[3] = -800;
	}
	else if(f_flag)//�رյ��ָ�
	{
		target_speed[3] =  800;
	}
	else //���ָǾ�ֹ
	{
		target_speed[3] = 0;
	}	
	motor_info[3].set_voltage = pid_calc(&motor_pid[3], target_speed[3], motor_info[3].rotor_speed);
}


static void magazine_init()
{
	pid_init(&motor_pid[3], 10 , 0.1, 0, 10000, 10000);
}

static void shoot_data_send()
{
	 
		set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage,motor_info[2].set_voltage,motor_info[3].set_voltage);
		 
    osDelay(1);		
}


//=============================================��������=============================================
static void heat_limit()
{
  if(infantry.heat_limit )
  {
	  if(infantry.shooter_heat > infantry.heat_limit )
	  {
		 	motor_info[2].set_voltage   = motor_info[2].set_voltage * 0;
		     
	  }
	  if(infantry.shooter_heat > infantry.heat_limit * 0.9)
	  {
			
		  	motor_info[2].set_voltage = motor_info[2].set_voltage * 0.5;
	  }
	  else
	  {
		//do nothing
	  }	  
  }
  else
  {
	  //do nothing
  }
}




