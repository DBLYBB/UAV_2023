/*
void UserTask_OneKeyCmd(void)//一键任务
{
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step;
	//////////////////////////////////////////////////////////////////////////////////
    //一键起飞 降落判断有遥控信号才执行
    if (rc_in.no_signal == 0)
    {
        //判断第6通道拨杆位置 1300<CH_6<1700
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            //还没有执行
            if (one_key_takeoff_f == 0)
            {
                //标记已经执行
                one_key_takeoff_f =
                    //执行一键起飞 
                    OneKey_Takeoff(100); //参数单位：厘米； 0：默认上位机设置的高度。
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_takeoff_f = 0;
        }
        //
        //判断第6通道拨杆位置 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //还没有执行
            if (one_key_land_f == 0)
            {
                //标记已经执行
                one_key_land_f =
                    //执行一键降落
                    OneKey_Land();
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_land_f = 0;
        }
	}
    ////////////////////////////////////////////////////////////////////////
	//一键锁桨
	if (rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1700 &&rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 2200) 
	{
		if (eme_stop == 0) 
		{
			eme_stop = 1;
			//执行一键锁桨
			FC_Lock();
			pwm_to_esc.pwm_m1 = 0;
			pwm_to_esc.pwm_m2 = 0;
			pwm_to_esc.pwm_m3 = 0;
			pwm_to_esc.pwm_m4 = 0;
		}
	} 
	else 
	{
		eme_stop = 0;
	}
	///////////////////////////////////////////////////////////////////////
	//任务启动
	if(rc_in.rc_ch.st_data.ch_[ch_7_aux3]>1700 && rc_in.rc_ch.st_data.ch_[ch_7_aux3]<2200)
		{
			//还没有执行
			if(one_key_mission_f ==0)
			{
				//标记已经执行
				one_key_mission_f = 1;
				//开始流程
				mission_step = 0;
			}
		}
		else
		{
			//复位标记，以便再次执行1
			one_key_mission_f = 0;	
			if(one_key_mission_f==1)OneKey_Land();			
		}
	//////////////////////////////////////////////////////////////////////
	//任务列表
	if(one_key_mission_f==1)
		{
			static u16 time_dly_cnt_ms;
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
						PID_height_init();
						
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
						mission_step += OneKey_Takeoff(100);
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
					PID_height_init();
					time_dly_cnt_ms = 0;
					mission_step += 1;
				}	
				break;
				case 5://开环前进移动2.2m 不出意外的话现在在A的正上方
				{
					if(time_dly_cnt_ms<10000)
					{
						tar_setdata(speed_x,0,height_set(ano_of.of_alt_cm,150),0);
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						tar_setdata(0,0,0,0);
						PID_height_init();
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 6://开环前进移动2.2m 不出意外的话现在在A的正上方
				{
					if(time_dly_cnt_ms<10000)
					{
						tar_setdata(-speed_x,0,height_set(ano_of.of_alt_cm,100),0);
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						pid_speed=0;
						tar_setdata(0,0,0,0);
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
}
*/
