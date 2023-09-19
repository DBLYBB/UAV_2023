/*
//////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////
	//任务列表
		if(one_key_mission_f==1)
		{
			static u16 time_dly_cnt_ms;
			static s16 integ_x,integ_y;
			static s16 integ_x_base,integ_y_base;
			static s32 pos_x_base,pos_y_base;
			static u16 icount=0;
			mission_flag=mission_step;
			//
			switch(mission_step)
			{
				case 0:
				{
					//reset
					time_dly_cnt_ms = 0;
					mission_step +=1;
				}
				break;
				case 1://解锁
					{
						PID_init();
						
						mission_step +=FC_Unlock();
					}
				break;
				case 2://十字处起飞
				{
					if(time_dly_cnt_ms<3500)//转桨延时
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += OneKey_Takeoff(150);
						one_key_takeoff_f=1;
					}
				}
				break;
				case 3://等三秒
				{
					if(time_dly_cnt_ms<3000)//任务延时
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 4://延时2s
				{
					pid_speed=0;
					PID_init();
					time_dly_cnt_ms = 0;
					mission_step += 1;
					integ_y_base=ano_of.intergral_y;
				}	
				break;
				case 5://右移0.5m 对准A
				{
					if(ano_of.intergral_y-integ_y_base>-50)
					{
						tar_setdata(0,-speed_y,height_set(ano_of.of_alt_cm,150),0);
					}
					else
					{
						integ_x_base=ano_of.intergral_x;
						integ_y_base=ano_of.intergral_y;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 6://开环前进移动2.2m 不出意外的话现在在A的正上方 
				{
					if(ano_of.intergral_x-integ_x_base<190)
					{
						tar_setdata(speed_x,xypid_set(ano_of.intergral_y-integ_y_base,0,speed_y),height_set(ano_of.of_alt_cm,150),0);
					}
					else
					{
						integ_x_base=ano_of.intergral_x;
						integ_y_base=ano_of.intergral_y;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 7://从A移动至开始点 启动激光笔
				{
					if(ano_of.intergral_x-integ_x_base<70) //接下来切换阶段由树莓派控制
					{
						tar_setdata(speed_x,xypid_set(ano_of.intergral_y-integ_y_base,0,speed_y),height_set(ano_of.of_alt_cm,150),0);
						mission_done_flag=0;
					}
					else
					{
						integ_y_base=ano_of.intergral_y;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_done_flag=1;
						if(received_data.next_task_sign==1)mission_step += 1;
					}
				}
				break;
				case 8://右移至边界，X向和yaw向速度由树莓派控制
				{
					if(ano_of.intergral_y-integ_y_base>-310)
					{
						tar_setdata(received_data.com_x,-speed_y,height_set(ano_of.of_alt_cm,150),received_data.com_yaw);
						mission_done_flag=0;
					}
					else
					{
						integ_x_base=ano_of.intergral_x;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_done_flag=1;
						if(received_data.next_task_sign==1)mission_step += 1;
					}
				}
				break;
				case 9://等三秒 
				{
					if(time_dly_cnt_ms<3000)//任务延时
					{
						tar_setdata(0,received_data.com_y,height_set(ano_of.of_alt_cm,150),received_data.com_yaw);
						time_dly_cnt_ms +=20;
						mission_done_flag=0;
					}
					else
					{
						integ_x_base=ano_of.intergral_x;
						tar_setdata(0,0,0,0);
						PID_init();
						mission_done_flag=1;
						time_dly_cnt_ms = 0;
						if(received_data.next_task_sign==1)mission_step += 1;
					}
				}
				break;
				case 10://后移至方块11，y向和yaw向速度由树莓派控制
				{
					if(ano_of.intergral_x-integ_x_base>-120)
					{
						tar_setdata(-speed_x,received_data.com_y,height_set(ano_of.of_alt_cm,150),received_data.com_yaw);
						mission_done_flag=0;
					}
					else
					{
						integ_y_base=ano_of.intergral_y;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_done_flag=1;
						if(received_data.next_task_sign==1)mission_step += 1;
					}
				}
				break;
				case 11://左移至方块13，x向和yaw向速度由树莓派控制
				{
					if(ano_of.intergral_y-integ_y_base<90)
					{
						tar_setdata(received_data.com_x,speed_y,height_set(ano_of.of_alt_cm,150),received_data.com_yaw);
						mission_done_flag=0;
					}
					else
					{
						integ_x_base=ano_of.intergral_x;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_done_flag=1;
						if(received_data.next_task_sign==1)mission_step += 1;
					}
				}
				break;
				case 12://后移至方块7，y向和yaw向速度由树莓派控制
				{
					if(ano_of.intergral_x-integ_x_base>-75)
					{
						tar_setdata(-speed_x,received_data.com_y,height_set(ano_of.of_alt_cm,150),received_data.com_yaw);
						mission_done_flag=0;
					}
					else
					{
						integ_y_base=ano_of.intergral_y;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_done_flag=1;
						if(received_data.next_task_sign==1)mission_step += 1;
					}
				}
				break;
				case 13://右移至方块5，x向和yaw向速度由树莓派控制
				{
					if(ano_of.intergral_y-integ_y_base>-80)
					{
						tar_setdata(received_data.com_x,-speed_y,height_set(ano_of.of_alt_cm,150),received_data.com_yaw);
						mission_done_flag=0;
					}
					else
					{
						integ_x_base=ano_of.intergral_x;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_done_flag=1;
						if(received_data.next_task_sign==1)mission_step += 1;
					}
				}
				break;
				case 14://后移至方块11，y向和yaw向速度由树莓派控制
				{
					if(ano_of.intergral_x-integ_x_base>-45)
					{
						tar_setdata(-speed_x,received_data.com_y,height_set(ano_of.of_alt_cm,150),received_data.com_yaw);
						mission_done_flag=0;
					}
					else
					{
						integ_y_base=ano_of.intergral_y;
					 	tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_done_flag=1;
						if(received_data.next_task_sign==1)mission_step += 1;
					}
				}
				break;
				case 15://左移至方块4，x向和yaw向速度由树莓派控制
				{
					if(ano_of.intergral_y-integ_y_base<140)
					{
						tar_setdata(received_data.com_x,speed_y,height_set(ano_of.of_alt_cm,150),received_data.com_yaw);
						mission_done_flag=0;
					}
					else
					{
						integ_x_base=ano_of.intergral_x;
						integ_y_base=ano_of.intergral_y;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_done_flag=1;
						if(received_data.next_task_sign==1)mission_step += 1;
					}
				}
				break;
				case 16://前进至方块18，y向和yaw向速度由树莓派控制
				{
					if(ano_of.intergral_x-integ_x_base<170)
					{
						tar_setdata(speed_x,xypid_set(ano_of.intergral_y-integ_y_base,0,speed_y),height_set(ano_of.of_alt_cm,150),0);
						mission_done_flag=0;
					}
					else
					{
						integ_y_base=ano_of.intergral_y;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_done_flag=1;
						if(received_data.next_task_sign==1)mission_step += 1;
					}
				}
				break;
				case 17://左移至A，x向和yaw向速度由树莓派控制
				{
					if(ano_of.intergral_y-integ_y_base<125)
					{
						tar_setdata(received_data.com_x,speed_y,height_set(ano_of.of_alt_cm,150),received_data.com_yaw);
						mission_done_flag=0;
					}
					else
					{
						integ_y_base=ano_of.intergral_y;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_done_flag=1;
						if(received_data.next_task_sign==1)mission_step += 1;
					}
				}
				break;
				case 18://左移至启动点前方
				{
					if(ano_of.intergral_y-integ_y_base<25)
					{
						tar_setdata(0,speed_y,height_set(ano_of.of_alt_cm,150),0);
						mission_done_flag=0;
					}
					else
					{
						integ_y_base=ano_of.intergral_y;
						integ_x_base=ano_of.intergral_x;
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 19://开环到启动点上方
				{
					if(ano_of.intergral_x-integ_x_base>-190)
					{
						tar_setdata(-speed_x,xypid_set(ano_of.intergral_y-integ_y_base,0,speed_y),height_set(ano_of.of_alt_cm,150),received_data.com_yaw);
						mission_done_flag=0;
					}
					else
					{
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 20://对位置降落
				{
					if(received_data.next_task_sign==0)
					{
						tar_setdata(received_data.com_x,received_data.com_y,height_set(ano_of.of_alt_cm,150),received_data.com_yaw);
						mission_done_flag=0;
					}
					else
					{
						tar_setdata(0,0,0,0);
						PID_init();
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				default:
				{
					OneKey_Land();
				}
				break;
			}
			
		}
		else
		{
			mission_step = 0;
		}
*/