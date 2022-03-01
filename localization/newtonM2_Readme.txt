1.工控机设置sudo usermod -a -G dialout a
2.串口
串口默认配置：
端口：com0
波特率：115200
数据位：8 
停止位：1
检验位：none

3.关闭COM0所有输出：
$cmd,output,com0,null*ff
4.获取网口设置：
$cmd,get,netpara*ff

返回命令：
$cmd,set,localip,192,168,10,4*ff 
$cmd,set,localmask,255,255,255,0*ff  
$cmd,set,localgate,192,168,10,1*ff 
$cmd,set,netipport,111,13,12,21,8000*ff 
$cmd,set,netuser, user:password *ff 
$cmd,set,mountpoint,BDKJ*ff


5.设置本地ip地址，用户密码：
 根据实际配置本地网络，如本地4G路由器网关地址为：192.168.3.1
 执行以下命令：
       $cmd,set,localip,192,168,3,10*ff 
       $cmd,set,localmask,255,255,255,0*ff  
       $cmd,set,localgate,192,168,3,1*ff 


6.设置连接ntrip服务器，千寻配置
  设置差分站主机ip，端口：
       $cmd,set,netipport,203,107,45,154,8002*ff
  设置差分站用户名、密码：
$cmd,set,netuser, qxnuxa003:89e69a8*ff 
  设置差分站stream名称：
	      $cmd,set,mountpoint,RTCM32_GGB*ff

7.设置连接ntrip服务器，本地基站配置
  设置差分站主机ip，端口：
       $cmd,set,netipport,218,15,23,202,8000*ff
  设置差分站用户名、密码：2~5
$cmd,set,netuser, xmjl2:xmjl2*ff 
  设置差分站stream名称：
	      $cmd,set,mountpoint,RTCM32*ff


8.GNSS航向设置
$cmd,set,headoffset,0*ff

9.双天线导航模式配置 
$cmd,set,navmode,FineAlign,off*ff
$cmd,set,navmode,coarsealign,off*ff
$cmd,set,navmode,dynamicalign,on*ff
$cmd,set,navmode,gnss,double*ff
$cmd,set,navmode,carmode,on*ff
$cmd,set,navmode,zupt,on*ff
$cmd,set,navmode,firmwareindex,0*ff

10.USB输出设置（如果使用USB）
$cmd,output,usb0,rawimub,0.010*ff
$cmd,output,usb0,inspvab,0.010*ff
$cmd,through,usb0,bestposb,1.000*ff
$cmd,through,usb0,rangeb,1.000*ff
$cmd,through,usb0,gpsephemb,1.000*ff
$cmd,through,usb0,gloephemerisb,1.000*ff
$cmd,through,usb0,bdsephemerisb,1.000*ff
$cmd,through,usb0,headingb,1.000*ff



11.设置杆臂补偿
    假设主天线距离M2主机距离为0.9米
$cmd,set,leverarm,gnss,0,-0.61,0.35*ff
新车更换位置后设置为
$cmd,set,leverarm,gnss,0,0.1,0.61*ff


12.输出GPFPD
$cmd,output,com0,gpfpd,0.1*ff     设置输出频率10Hz

13.输出GTIMU
$cmd,output,com0,gtimu,0.02*ff    设置输出频率50Hz

14.保存配置
  $cmd,save,config*ff

15.重启设备
