#25年智能救援
===
  电控：陈树泽
  
  电控：王博宣
  
  视觉：庄裕辉
  
  结构：王家豪
  
  
`电控`：
===
  该工程由hal库配置的，基于freertos操作系统，且有激光测距模块MS53L0M（正点原子）,HWT101,JY901B,与树莓派通信，无线通讯NRF2401模块驱动,且拥有自主绘制并且使用的遥控器
  
  电控总结：<br>
  
1.在一个消耗电量大的项目中，调试的时候一定不能（或避免）只使用stlink供电，在这次比赛中，由于这个问题导致烧了两块激光测距模块（舵机多，电流不稳定）<br>
2.由于技术不足，在赛前没有意识到堆栈溢出的情况，在最后赛前准备时，小车会开着开着就卡住不动，在这个问题上卡了两天，一开始还以为是电池功率不够不能支撑开局的野牛加速<br>
3.大赛经验不足，在第一次因碰撞对方而堵死，来到最后的第二次保底机会下，应该想到告知视觉只抓红色或蓝色，减少意外情况的发生<br>
4.没有为舵机单独设计电源，导致主板电流波动过大<br>

'代码'<br>
=
 *#方便移植的有*<br>
 
 1.手搓激光测距模块MS53L0M通信代码<br>
 （此模块使用时要多注意电流，尽可能不要只使用stlink给开发板供电，容易烧坏，现象是原本为一发一收的状态，但在仅仅使用stlink供电后，模块会回归到默认（一直发送），并且无论向模块发送任何数据，都不能得到他的响应，即便是上位机控制也不行）<br>
 2.手搓的定位系统以及寻点代码<br>
 目前只能做到将x，y分别存放到不同的x，y数组，然后小车会'直线'(水平不足，非曲线)前往该地点，当距离该点在一定范围内，则进入下个点的前进<br>
 且在师兄的建议下，加入了相对位置的概念，可以做到到达一个点后，通过相对位置前往想要去的地方，在陀螺仪较为精准的情况下，可以减少累计误差<br>
 3.能够使用的NRF2041从机代码以及与搭载该模块的遥控器通信的主机代码<br>
 4.HAL库配置的 空闲中断 HWT101通信代码，以及jy901s通信代码<br>

 *代码详细*<br>
 1.定位
 
``` 
 void X_AND_Y_GET(float *x, float *y)
{
    float sa1_real, sa2_real;
   
    sa_angle = pi * HWT101.angle / 180.0f;

    READ_SPEED();

    sa1_real = location.speed_l_now * 6.5f * pi / 4680.0f;//6.5为轮子直径6.5cm，4680是轮子转一圈的总脉冲数，
    sa2_real =  location.speed_R_now * 6.5f * pi / 4680.0f;

    location.y += (cos(sa_angle) * sa1_real + cos(sa_angle) * sa2_real) / 2.0f;//基本定位
    location.x += (-sin(sa_angle) * sa1_real - sin(sa_angle) * sa2_real) / 2.0f;
//以下的是作为相对为位置读取
	location.y_relative= location.y-location.y_record;
	location.x_relative= location.x-location.x_record;
	
	location.safe_x_relative=location.safe_x- location.safe_x_Record;//record是记录值，当立下一个相对位置初始点时候，将此刻的location.safe_y_Record=location.y，也就是得到坐标系偏移量（x同理）
	location.safe_y_relative=location.safe_y- location.safe_y_Record;
	//得到了每一次的定点的相对位置
	location.safe_x_one_relative=location.safe_x_one-location.safe_x_one_Record;
	location.safe_y_one_relative=location.safe_y_one- location.safe_y_one_Record;
	location.safe_x_two_relative=location.safe_x_two- location.safe_x_two_Record;
	location.safe_y_two_relative=location.safe_y_two- location.safe_y_two_Record;
	
	
    *x = (float)location.x;
    *y = (float)location.y;
}

void Location_GO_EYSE(float expect_x, float expect_y, float current_x, float current_y,int motor) // 输入目标位置xy，输入当前位置，
{
    sa_dx = current_x - expect_x;
    sa_dy = current_y - expect_y;

    sa_aerfa = atan2(sa_dx, sa_dy) * 180.0f / pi;

    sa_yaw4 = -180.0f - sa_aerfa;

    if (sa_yaw4 < -180)
    {
        sa_yaw4 += 360;
    }
    sa_L = sa_dx * sa_dx + sa_dy * sa_dy; // 使用平方可以避免负号的存在

    if (sa_L >= 5)//5可通过实际更改
    {
        straight_line(motor + sa_L / 10, motor + sa_L / 10, sa_yaw4, &sa_20, &sa_21);//将计算后的角度导入到速度环串偏航环的运动中，sa_yaw4是接下来要前往的偏航角度，范围为-180 - 180
    }

    if (sa_L < 5)
    {
        Motor_370_respectively(0, 0);
    }
}

int location_goto_point(float target_x, float target_y,int option,int motor)//输入目标位置，
{
    //static int car_y = 4.0f; // 灏忚溅 y 鏂瑰悜鍋忕Щ
    //此刻引入了记录值，也就是使用了相对位置，当不想要使用相对位置时，将record置0 
   target_difference =((( (target_x-location.x_record )- location.x_Relative) * ((target_x-location.x_record) - location.x_Relative) )+ ((target_y-location.y_record ) - location.y_Relative) * ((target_y-location.y_record ) - location.y_Relative));//注意此处的括号，不可删减
    if (target_difference > 100.0f)//距离该点够近
    {
       Location_GO_EYSE(target_x-location.x_record, target_y -location.y_record, location.x_Relative, location.y_Relative,motor);
        return 0;
    }
   else
    {
		
		if(option==1)
		{
		 GO_BELOW_EIGHT_SATUS++;
			
        return 1;
		}
		if(option==2)
		{
		   GO_BELOW_EIGHT_SATUS_RETURN++;
		  return 1;  
		
		
		}
		
			
			
		return 1; 
    }
	
	
}
float g_location_x_new_return[100] = { 57.0f,  80.0f,   80.0f,  78.0f, 78.0f};     
float g_location_y_new_return[100] = { -40.0f, -140.0f, -190.f,140.0f, 140.0f};    
                                                                             
void GO_BELOW_EIGHT_return(void)  
{
	
     while (location_goto_point(g_location_x_new_return[GO_BELOW_EIGHT_SATUS_RETURN]+location.x_record+location_offset_Return, g_location_y_new_return[GO_BELOW_EIGHT_SATUS_RETURN]+location.y_record,2,6000) == 0)
        {
			
			   return;
           
			
        }




}


```

 

 

    
           
           
           
  

  
