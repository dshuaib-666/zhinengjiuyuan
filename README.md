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
 （此模块使用时要多注意电流，尽可能不要只使用stlink给开发板供电，容易烧坏，现象是原本为一发一收的状态，但在仅仅使用stlink供电后，模块会回归到默认（一直发送），并且无论像模块发送任何数据，都得到他的响应，即便是上位机控制也不行）<br>
 2.手搓的定位系统以及寻点代码<br>
 目前只能做到将x，y分别存放到不同的x，y数组，然后小车会'直线'(水平不足，非曲线)前往该地点，当距离该点在一定范围内，则进入下个点的前进<br>
 3.能够使用的NRF2041从机代码以及与搭载该模块的遥控器通信的主机代码<br>
 4.HAL库配置的 空闲中断 HWT101通信代码，以及jy901s通信代码<br>

 *代码详细*<br>
 1.
 
``` 
 void X_AND_Y_GET(float *x, float *y)
{
    float sa1_real, sa2_real;
   
    sa_angle = pi * HWT101.angle / 180.0f;

    READ_SPEED();

    sa1_real = location.speed_l_now * 6.5f * pi / 4680.0f;
    sa2_real =  location.speed_R_now * 6.5f * pi / 4680.0f;

    location.y += (cos(sa_angle) * sa1_real + cos(sa_angle) * sa2_real) / 2.0f;//基本定位
    location.x += (-sin(sa_angle) * sa1_real - sin(sa_angle) * sa2_real) / 2.0f;
//以下的是作为相对为位置读取
	location.y_relative= location.y-location.y_record;
	location.x_relative= location.x-location.x_record;
	
	location.safe_x_relative=location.safe_x- location.safe_x_Record;
	location.safe_y_relative=location.safe_y- location.safe_y_Record;
	//得到了每一次的定点的相对位置
	location.safe_x_one_relative=location.safe_x_one-location.safe_x_one_Record;
	location.safe_y_one_relative=location.safe_y_one- location.safe_y_one_Record;
	location.safe_x_two_relative=location.safe_x_two- location.safe_x_two_Record;
	location.safe_y_two_relative=location.safe_y_two- location.safe_y_two_Record;
	
	
    *x = (float)location.x;
    *y = (float)location.y;
}

```

 

 

    
           
           
           
  

  
