/*
//////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////
	//�����б�
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
				case 1://����
					{
						PID_init();
						
						mission_step +=FC_Unlock();
					}
				break;
				case 2://ʮ�ִ����
				{
					if(time_dly_cnt_ms<3500)//ת����ʱ
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
				case 3://������
				{
					if(time_dly_cnt_ms<3000)//������ʱ
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
				case 4://��ʱ2s
				{
					pid_speed=0;
					PID_init();
					time_dly_cnt_ms = 0;
					mission_step += 1;
					integ_y_base=ano_of.intergral_y;
				}	
				break;
				case 5://����0.5m ��׼A
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
				case 6://����ǰ���ƶ�2.2m ��������Ļ�������A�����Ϸ� 
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
				case 7://��A�ƶ�����ʼ�� ���������
				{
					if(ano_of.intergral_x-integ_x_base<70) //�������л��׶�����ݮ�ɿ���
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
				case 8://�������߽磬X���yaw���ٶ�����ݮ�ɿ���
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
				case 9://������ 
				{
					if(time_dly_cnt_ms<3000)//������ʱ
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
				case 10://����������11��y���yaw���ٶ�����ݮ�ɿ���
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
				case 11://����������13��x���yaw���ٶ�����ݮ�ɿ���
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
				case 12://����������7��y���yaw���ٶ�����ݮ�ɿ���
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
				case 13://����������5��x���yaw���ٶ�����ݮ�ɿ���
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
				case 14://����������11��y���yaw���ٶ�����ݮ�ɿ���
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
				case 15://����������4��x���yaw���ٶ�����ݮ�ɿ���
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
				case 16://ǰ��������18��y���yaw���ٶ�����ݮ�ɿ���
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
				case 17://������A��x���yaw���ٶ�����ݮ�ɿ���
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
				case 18://������������ǰ��
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
				case 19://�������������Ϸ�
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
				case 20://��λ�ý���
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