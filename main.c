/*2DOF manipulator program*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <process.h>
#include <sync.h>
#include <sys/dispatch.h>
#include <sys/siginfo.h>
#include <sys/neutrino.h>
#include <hw/inout.h>
/*        Added functions                                                 */
#include "../include/config.h"
#include "../include/params.h"
#include "../include/function.h"
#include "../H_DRIVER/DA/DA_driver.h"
#include "../include/timer.h"
#include "../H_DRIVER/CNT/CNT_driver.h"
#include "../H_DRIVER/AD/AD_driver.h"
#include "../COMM/command_server.h"

struct   SEN526_104_DA  *da;
struct   SEN526_104_CNT *cnt;
struct   SEN526_104_AD  *ad;

struct path    Path;
struct params  Motorp;
struct status  Cur_j, Des_j, Com_j;         /* joint coordinate-system     */
struct status  Cur_r, Des_r, Com_r;         /* robot coordinate-system     */
struct status  Cur_e, Des_e, Com_e;         /* absolute coordinate-system  */


enum mode {nop,robot_control,robot_control_sbmpc};

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


struct params  Motorp;

/*    variables used in control funtion{state varaibles}			     */

int            ctrlEndFlag = 0;
int            ctrltrig    = OFF;
int            logstrig    = OFF;
int            logdtrig    = OFF;
int            log_enable  = 0;
int            RESET       = 0;
int            traj_flag   = 0;
int            State_Length = 73;

/*             variables used in logging data                              */

double         logbuff1[15][logsMax];
double         COM_joint_torque[2];
double         COM_volt[2];
double         COM_torque[2]={0.,0.};
double         SENSE_current[2];
double         SENSE_voltage[2];

//double		   GDfinal;
double         DESTime      = 0 ;
double         DESJ_POS[2]  = {0.,0.};
double         START_POS[2]  = {0.,0.};
double         MEAN_current  = 0.;
unsigned long  logtime       = 0;

/*SBMPC									*/
double ROBOT_COMMANDS[250][6];


void init(void)
{

     ThreadCtl(_NTO_TCTL_IO,0);

     if ((chid = ChannelCreate (0)) == -1)
        {
         fprintf (stderr,"timer.c: couldn't create channel!\n");
         perror (NULL);
         exit (EXIT_FAILURE);
        }
     setupTimer();
     initializeAll();
     Motorp.mode = nop;
}

int initializeAll(void)
{
   if(( da  = daOpen(BASE_ADDR5261))   == NULL) return ERROR;
   if(( cnt = cntOpen(BASE_ADDR5261))  == NULL) return ERROR;
   if(( ad  = adOpen(BASE_ADDR5261))   == NULL) return ERROR;

     initializeData();
     return(OK);
}

void initializeData(void)
{
}

/*************************************************************************/
/*                                                                       */
/*        start ( ) -  starts timer for control                          */
/*                                                                       */
/*************************************************************************/

void  start(void)
{
     timer_settime (timerid, 0, &timer, NULL);
     ctrltrig = 1;
     ctrlEndFlag = 1;
     ctrlTask(&Motorp);
     return;
}


/***************************************************************************/
/*        ctrlTask ( struct params *motor )                                */
/*                                                                         */
/*        --- explanation ---                                              */
/*        After clock signal(i.e. can take control semaphore),             */
/*        take semaphore and excute control().                             */
/*                                                                         */
/*        --- called ---                                                   */
/*        start()                                                          */
/***************************************************************************/


int ctrlTask(struct params *motor)
{

     int             rcvid;
     MessageT        msg;
     static          unsigned long ticks = 0;
     pthread_attr_t  attr;

     ctrlEndFlag = 1;
     pthread_attr_init( &attr );
     pthread_attr_setdetachstate( &attr,PTHREAD_CREATE_DETACHED);

     while(ctrltrig)
     {
        rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
        if(rcvid == 0)
	  {
            if(!ctrlEndFlag)
	      {
                printf("ERROR : Control error occured.\n        fin\n");
                fin();
                return (ERROR);
              }

            ctrlEndFlag = 0;
            pthread_create( NULL, &attr,control,motor);
            ticks++;
          }
     }

    return (EXIT_SUCCESS);
}

int endTask( void )
{
    ctrlEndFlag = 1;
    pthread_mutex_unlock( &mutex );
    return (EXIT_SUCCESS);
}


/**************************************************************************/
/*                                                                        */
/*                      Control Function                                  */
/*                                                                        */
/*                                                                        */
/**************************************************************************/

void control(struct params *motor)
{
     static struct params    param;
     static        int       mode = nop;
     static        int       trig;

     if( pthread_mutex_lock ( &mutex ) != EOK )
       {
        printf(" ERROR : mutex cannnot lock.\n fin\n");
        fin();
        return;
       }
     memcpy(&param, motor, sizeof(struct params));
     if(param.trig)
       {
	 trig = TRUE;
	 motor->trig = FALSE;
       }
     else
       trig = FALSE;
       mode = param.mode;
       getCurrentStatus();

      switch(mode){

      case robot_control:
	   robot_control_func(trig);

     break;

     case robot_control_sbmpc:
	  robot_control_sbmpc_func();
	break;


     default:
       Nop();
       break;
     }


}

/***************************************************************************/
/*        Nop ( void )                                                     */
/*                                                                         */
/*        --- called ---                                                   */
/*        control()                                                        */
/***************************************************************************/

void Nop(void)
{
  TorqCtrl();
  endTask();
}

void robot_control_sbmpc_func(void)
{
static unsigned long ticks=0;
static unsigned long ticks1=0;
static unsigned long i=0;

if  ((ticks%20)==0)
{

    i++;
    ticks1=0;


}



if (i<State_Length)
{
  Com_j.pos[0] = ROBOT_COMMANDS[i-1][0]  + (ROBOT_COMMANDS[i][0]-ROBOT_COMMANDS[i-1][0])/20.*ticks1;
  Com_j.vel[0] = ROBOT_COMMANDS[i-1][1]  + (ROBOT_COMMANDS[i][1]-ROBOT_COMMANDS[i-1][1])/20.*ticks1;
  Com_j.acc[0] = ROBOT_COMMANDS[i-1][2]  + (ROBOT_COMMANDS[i][2]-ROBOT_COMMANDS[i-1][2])/20.*ticks1;

  Com_j.pos[1] = ROBOT_COMMANDS[i-1][3]  + (ROBOT_COMMANDS[i][3]-ROBOT_COMMANDS[i-1][3])/20.*ticks1;
  Com_j.vel[1] = ROBOT_COMMANDS[i-1][4]  + (ROBOT_COMMANDS[i][4]-ROBOT_COMMANDS[i-1][4])/20.*ticks1;
  Com_j.acc[1] = ROBOT_COMMANDS[i-1][5]  + (ROBOT_COMMANDS[i][5]-ROBOT_COMMANDS[i-1][5])/20.*ticks1;

}
else{
 Com_j.vel[0] = 0.;
 Com_j.vel[1] = 0.;
 Com_j.acc[0] = 0.;
 Com_j.acc[1] = 0.;

}

//Com_j.pos[0]=0;
//Com_j.vel[0]=0;
//Com_j.acc[0]=0;


if ((ticks%1000)==0)
    printf("CMD Joint Pos 0 =  %f\n",Com_j.pos[0]);



TorqCtrl();
logSave1();
ticks1++;
ticks++;

endTask();

}


void robot_control_func(int trig)
{
static unsigned long int ticks=0;


if ((ticks%1000)==0)
printf("Joint 1 = %f \t Joint 2 = %f\n",Com_j.pos[0],Com_j.pos[1]);
traCtrl();
ticks++;
//endTask();
return;

}





/*Initialize desired path -> get coefficients
  of a 5th ordered trajectory */

void pathInit(double Lstart[2], double destination[2], double Dtime)
{
    int i =0;
    double t3, t4, t5;
    double diff[2];

    t3 = Dtime * Dtime * Dtime;
    t4 = t3 * Dtime;
    t5 = t4 * Dtime;

    i = 0;

      for(i=0;i<2;i++)
      {


        diff[i] = destination[i] - Lstart[i];

        Path.pos[i][0] =    Lstart[i];
        Path.pos[i][3] =    10. * diff[i] / t3;
        Path.pos[i][4] =   -15. * diff[i] / t4;
        Path.pos[i][5] =     6. * diff[i] / t5;

        Path.vel[i][2] = 3. * Path.pos[i][3];
        Path.vel[i][3] = 4. * Path.pos[i][4];
        Path.vel[i][4] = 5. * Path.pos[i][5];
      }

        Path.time = Dtime;


    return;
}

/*This function generates position, velocity, and acceleration based on a given time,
  which is between 0 and Tmax. It uses the coefficients from the pathInit*/

void pathGenerate(unsigned long Dtime)
{
    int i;
    double t;
	static double prev_Com_j_vel=0.;

    t = (Dtime * TICKS);
    t = (t > Path.time) ? Path.time : t;

    for (i=0;i<2;i++){
        Com_j.pos[i] = Path.pos[i][0]
            + t * t * t * (Path.pos[i][3]
            + t * (Path.pos[i][4] + t * Path.pos[i][5]));

        Com_j.vel[i] = t * t * (Path.vel[i][2]
            + t * (Path.vel[i][3] + t * Path.vel[i][4]));

        Com_j.acc[i]  = (Com_j.vel[i] - prev_Com_j_vel) / TICKS;
		prev_Com_j_vel = Com_j.vel[0];
    }


	return;
}



void traCtrl(void)
{
    int i;
   // static double Lstart, destination, desTime;

    static unsigned long tick = 0;
	static int trig  = 1;
	static int state = 0;

    if (traj_flag==1)
	{
       // Lstart          = Com_j.pos[0];
       // destination     = -PI/2.0;
		//desTime         = 10.;
        tick = 0;
        pathInit(START_POS,DESJ_POS,DESTime);
		traj_flag  = 0;
		state = 1;
        endTask();
        return;
	}

	pathGenerate(tick);
    TorqCtrl();
    tick++;
    endTask();

    return;
}

/*This function determines the correponding torque based on positon, velocility, and acceleration.*/

void TorqCtrl(void)
{


 static unsigned long ticks = 0;

 float g   = 9.81;
 float m1  = 2.883;
 float lc1 = 0.195;
 float l1  = 0.37;
 float I1  = 0.034;  // need to solve this one
 float m2  = 2.4;
 float lc2 = 0.2244;
 float l2  = 0.3;
 float I2  = 0.029; // need to get this one


 double accel[2]    =  {0.,0.};
 double torq[2]     =  {0.,0.};
 double volt[2]     =  {0.,0.};
 double Torqconst   =   1.127823916;							/* N-m/V 65.72222*30.2m*0.6413 */
 double gravity[2]  =  {0.,0.};
 double friction[2] =  {0.,0.};
 double brk_force[2]=  {0.35,0.35};
 int    i=0;
    /*controller parameters loaded */
 double Kd[2]       =  {50.0,50.0};
 double Kp[2]       =  {2000.,2000.0};
 double M11 = 0;
 double M12 = 0;
 double M21 = 0;
 double M22 = 0;

 double C11 = 0;
 double C12 = 0;
 double C21 = 0;
 double C22 = 0;
 double q2  = 0;
 double dq1 = 0;
 double dq2 = 0;

  // Com_j.pos[1] = Com_j.pos[0];
 //  Com_j.vel[1] = Com_j.vel[0];
 //  Com_j.acc[1] = Com_j.acc[0];

    q2   = Com_j.pos[1];
    dq1  = Com_j.vel[0];
    dq2  = Com_j.vel[1];


     M11 = m1*lc1*lc1  + m2*(l1*l1 + lc2*lc2 + 2*l1*lc2*cos(q2)) + I1 + I2;
     M12 = m2*(lc2*lc2 + l1*lc2*cos(q2)) + I2;
     M21 = m2*(lc2*lc2 + l1*lc2*cos(q2)) + I2;
     M22 = m2*lc2*lc2  + I2;

     C11 = -m2*l1*lc2*sin(q2)*dq2;
     C12 = -m2*l1*lc2*sin(q2)*(dq1 + dq2);
     C21 =  m2*l1*lc2*sin(q2)*dq1;
     C22 =  0;


     gravity[0]  = (m1*lc1 + m2*l1)*g*sin(Com_j.pos[0]) +
                      m2*lc2*g*sin(Com_j.pos[0] + Com_j.pos[1]);

     gravity[1]  =  m2*lc2*g*sin(Com_j.pos[0]+ Com_j.pos[1]);

     accel[0]    = Com_j.acc[0] + Kd[0] * ( Com_j.vel[0] - Cur_j.vel[0] )
	                   + Kp[0] * ( Com_j.pos[0] - Cur_j.pos[0]);

     accel[1]    = Com_j.acc[1] + Kd[1] * ( Com_j.vel[1] - Cur_j.vel[1] )
	                  + Kp[1] * ( Com_j.pos[1] - Cur_j.pos[1]);


   //  torq[0]     = gravity[0];
   // torq[1]     = gravity[1];


    // torq[0]  =  M11*accel[0] + M12*accel[1]  + C11*dq1 + C12*dq2 + gravity[0];
   //  torq[1]  =  M21*accel[0] + M22*accel[1]  + C21*dq1 + C22*dq2 + gravity[1];

     torq[0]  =   M11*accel[0] +  C11*dq1 + gravity[0];
     torq[1]  =   M22*accel[1] +  C22*dq2 + gravity[1];



      volt[0]  =  torq[0]/Torqconst;
      volt[1]  =  torq[1]/Torqconst;

       if(volt[0] >  9.99)  volt[0] =  9.99;
       if(volt[0] < -9.99)  volt[0] = -9.99;


       if(volt[1] >  9.99) volt[1] =  9.99;
       if(volt[1] < -9.99) volt[1] = -9.99;




      COM_torque[0] = volt[0]*Torqconst;
      COM_torque[1] = volt[1]*Torqconst;









	output(volt);
    ticks++;



return;
}



/***************************************************************************/
/*        Output ( void )                                                  */
/*        Output the computed voltage to the DA                            */
/*                                                                         */
/*        TorqCtrl()                                                       */
/***************************************************************************/

void output(double volt[])
{

     double buff[2] = { 0., 0.};

  //  volt[0] = 2.0;
   //  volt[1] = 2.0;
     buff[0] =    volt[0];
     buff[1] =   -volt[1];

     daOut(da,buff);
     return;
}



/*Trajectory filter */
void bessel(double in[6], double out[6])
{
  int i;
  static double prein[6] = {0.,0.,0.,0.,0.,0.};
  static double preout[6] = {0.,0.,0.,0.,0.,0.};
  double CT[6];
 // const double Cutoff[4] = {100.,100.,100.,100.};
  const double Cutoff[6] = {100.,100.,1000.,100.,100.,100.};

  for(i=0;i<6;i++){
    CT[i] = Cutoff[i]*20.*TICKS;

    out[i] = (CT[i] * (in[i] + prein[i])
              - (CT[i] - 2.) * preout[i])/(CT[i] + 2.);

    prein[i] = in[i];
    preout[i] = out[i];
  }
}




/***************************************************************************/
/*        getCurrentStatus ( void )                                        */


void getCurrentStatus(void)
{

     int     i;
     long    *cntbuff;
     double  prev_Cur_j_pos[4] = {0.,0.,0.,0.};
     double  *addata;
     double  temp_buff[4]={0.,0.,0.,0.};
     double  prev_Yaw    =0.;




     for(i=0;i<2;i++)
       {
	 prev_Cur_j_pos[i] = Cur_j.pos[i];
       }
       cntbuff=cntRead(cnt);


         Cur_j.pos[0] = -count2angle(*(cntbuff + 0));
	     Cur_j.vel[0] = (Cur_j.pos[0] - prev_Cur_j_pos[0]) / TICKS;
	     Cur_j.pos[1] = count2angle(*(cntbuff + 1));
	     Cur_j.vel[1] = (Cur_j.pos[1] - prev_Cur_j_pos[1]) / TICKS;

		 addata=adRead(ad);
            for (i=0;i<2;i++)
             temp_buff[i]=*(addata+i);
										    // sense_voltage[0]=*(addata + 1);
	 //    bessel(temp_buff,temp_buff);
         SENSE_current[0] =  temp_buff[0];
	     SENSE_current[1] =  temp_buff[1];




	return;
}


/***************************************************************************/
/*        fin ( void )                                                     */
/*                                                                         */
/*        --- explanation ---                                              */
/*        End Command ...                                                  */
/*                                                                         */
/*        - Stop Timer                                                     */
/*        - Close Drivers                                                  */
/*        - Delete Tasks                                                   */
/*        - Delete Semaphore(s)                                            */
/***************************************************************************/

void fin(void)
{
    printf("Finishing program execution .... \n");
    Motorp.mode = nop;
    zeroout(da);
    ctrltrig = 0;
    logdtrig = 0;
    logstrig = 0;
    timer_delete(timerid);
    daClose(da);
    cntClose(cnt);
    logWrite1();
    return;

}

void logSaveCal(void)
{
  logtime++;
    if(logtime < logsMax)
		{

    /*********** time  ****************/
    logbuff1[0][logtime] = logtime-1;
    /*********** joint angle **********/

			logbuff1[1][logtime] =  Com_j.vel[0];
			logbuff1[2][logtime] =  MEAN_current;
			logbuff1[3][logtime] =  Cur_j.pos[0];
			logbuff1[4][logtime] =  0.;
			logbuff1[5][logtime] =  0.;
			logbuff1[6][logtime] =  0.;
			logbuff1[7][logtime] =  0.;
			logbuff1[8][logtime] =  0.;
			// sense current 1 V/A
																     // sense current x 30.2 mNm /A * 66
		}
    return;
}

void logSave1(void)
{

    logtime++;

    if(logtime < logsMax)
		{

    /*********** time  ****************/
    logbuff1[0][logtime] = logtime-1;
    /*********** joint angle **********/

			logbuff1[1][logtime] =  Com_j.pos[0];
			logbuff1[2][logtime] =  Com_j.vel[0];
			logbuff1[3][logtime] =  Com_j.acc[0];
			logbuff1[4][logtime] =  Com_j.pos[1];
			logbuff1[5][logtime] =  Com_j.vel[1];
			logbuff1[6][logtime] =  Com_j.acc[1];
			logbuff1[7][logtime] =  Cur_j.pos[0];
			logbuff1[8][logtime] =  Cur_j.vel[0];
			logbuff1[9][logtime] =  Cur_j.acc[0];
			logbuff1[10][logtime] =  Cur_j.pos[1];
			logbuff1[11][logtime] =  Cur_j.vel[1];
			logbuff1[12][logtime] =  Cur_j.acc[1];
			logbuff1[13][logtime] =  COM_torque[0];
			logbuff1[14][logtime] =  COM_torque[1];



			//logbuff1[7][logtime] =  -SENSE_current[0]/.4* 30.2/1000*65.722;  // 0.4 V/A; Sense Joint Torque
			//logbuff1[8][logtime] =   0.;
			// sense current 1 V/A
																            // sense current x 30.2 mNm /A * 66
		}
    return;
}

void logWrite1(void)
{

    int i;
    char *filename;
    FILE *file;
    time_t time_of_day;

    printf("\n\t Saving Data ...\n");
    time_of_day = time( NULL );
    filename = "data.txt";

    if((file = fopen(filename,"w")) == NULL)
	{
         printf("file open error!\n");
         exit(1);
	}
   for(i=1;i<logtime;i++)
	{
                fprintf(file,
                "%7f %7f %7f %7f %7f %7f %7f %7f %7f %7f %7f %7f %7f %7f %7f\n",
	        logbuff1[0][i],
	        logbuff1[1][i],logbuff1[2][i],logbuff1[3][i],logbuff1[4][i],logbuff1[5][i],logbuff1[6][i],
	        logbuff1[7][i],logbuff1[8][i],logbuff1[9][i],logbuff1[10][i],logbuff1[11][i],logbuff1[12][i],
	        logbuff1[13][i],logbuff1[14][i]);

	}
    fclose(file);
    printf("\n\t Data Saved Successfully \n\n");
}


void read_commands(void)
{
  char *filename;
  FILE *file;
  int i = 0;
  float robot_commands[250][6]={};
  float temp;
  double filtered_com[6] = {0.,0.,0.,0.,0.,0.};
  double filtered_com1[6] = {0.,0.,0.,0.,0.,0.};

  float prev_vel=0;

  filename = "states25lb.txt";



    if((file = fopen(filename,"r")) == NULL)
	{
         printf("file open error!\n");
         exit(1);
	}

/*dynamics*/
  for(i=0;i<State_Length;i++)
	{
	  fscanf(file,"%f %f %f %f %f %f %f %f %f %f %f %f",&robot_commands[i][0],&robot_commands[i][1],&robot_commands[i][2],&robot_commands[i][3],&robot_commands[i][4],&robot_commands[i][5],&temp,&temp,&temp,&temp,&temp,&temp);

	  filtered_com[0]= robot_commands[i][0];
	  filtered_com[1]= robot_commands[i][1];
	  filtered_com[2]= robot_commands[i][2];
	  filtered_com[3]= robot_commands[i][3];
	  filtered_com[4]= robot_commands[i][4];
	  filtered_com[5]= robot_commands[i][5];

	  //bessel(filtered_com,filtered_com);

	  robot_commands[i][0] = filtered_com[0];
	  robot_commands[i][1] = filtered_com[1];
	  robot_commands[i][2] = filtered_com[2];
	  robot_commands[i][3] = filtered_com[3];
	  robot_commands[i][4] = filtered_com[4];
	  robot_commands[i][5] = filtered_com[5];


	  ROBOT_COMMANDS[i][0] = robot_commands[i][0];
	  ROBOT_COMMANDS[i][1] = robot_commands[i][1];
      ROBOT_COMMANDS[i][2] = robot_commands[i][2];
      ROBOT_COMMANDS[i][3] = robot_commands[i][3];
	  ROBOT_COMMANDS[i][4] = robot_commands[i][4];
      ROBOT_COMMANDS[i][5] = robot_commands[i][5];

	//  printf("%d\t %f\t%f\n",i,ROBOT_COMMANDS[i][0],ROBOT_COMMANDS[i][1]);
	}

  printf("Done ... \n");

  fclose(file);
}


void read_robot_commands(void)
{
read_commands();
Motorp.mode = robot_control_sbmpc;
Motorp.trig = TRUE;

}

void move_robot(double prms[])
{


	//double GDfinal, GDtime;
	if ((fabs(prms[0])<(2*PI))  && (prms[2] > 0.))
	{
		DESJ_POS[0]      = prms[0];
		DESJ_POS[1]      = prms[1];
        DESTime          = prms[2];
        START_POS[0]     = Com_j.pos[0];
        START_POS[1]      = Com_j.pos[1];

		Motorp.mode = robot_control;
		Motorp.trig = TRUE;
		traj_flag   = 1;
	}
	else /* do nothing */
	{
           DESJ_POS[0]   = Com_j.pos[0];
           DESJ_POS[1]   = Com_j.pos[1];
           START_POS[0]  = Com_j.pos[0];
           START_POS[1]  = Com_j.pos[1];
	       DESTime       = 1.;
	       traj_flag  = 1;
	}

	printf("Des Pos J0 = %f\tDes Pos J1 = %f  Des Time = %f \n",DESJ_POS[0], DESJ_POS[1], DESTime);


}



void init_robot(void)
{
	    Motorp.mode = robot_control;
		Motorp.trig = TRUE;



	//printf("Desired Position = %f\tDesired Time = %f\n", GDfinal,GDtime);
}




int main(void)
{
init();
pthread_create( NULL,NULL,&start,NULL);
command_server();
return(OK);
}



