## 当回调函数写在类中，如何写subscribe的回调函数

```cpp
class ChargeInfo
{
public:
    ChargeInfo(){
        pub = nh.advertise<charge_ppn::ChargeData>("charge_data",10);
        //这里要注意，InfoCallback是ChargeInfo类中的函数，所以在定义回调函数时应使用域修饰符::加上类空间，以及加上this引用。
        sub=nh.subscribe("/BMS_status", 10 , &ChargeInfo::InfoCallback,this); 
    }
    void InfoCallback(const scout_msgs::ScoutBmsStatus::ConstPtr& battery_msgs);
}
```
