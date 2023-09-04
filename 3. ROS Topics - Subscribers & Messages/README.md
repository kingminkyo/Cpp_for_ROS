# ROS TOPIC: Subscribers & Messages


## 이 장에서 앞서 생략한 내용
1. roscpp와 std_msgs 의존성을 사용하는 topic_subscriber_pkg 작성
2. 스크립트 cpp파일 및 launch파일 작성


- Subscriber는 Topic에서 정보를 읽는 노드이다.


> simple_topic_subscriber.cpp

``` cpp
#include <ros/ros.h>
#include <std_msgs/Int32.h>

void counterCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("%d", msg->data); //메시지 출력
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_subscriber"); //노드 생성
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("counter", 1000, counterCallback); //매 루프마다 callback을 
                                                                          //호출하는 Subscriber 객체 생성
    
    ros::spin(); // 루프 생성
    
    return 0;
}
```

- 해당 코드는 counter 토픽을 통해 메시지를 callback한다.

- 위 코드를 실행시킨 후 echo를 통해 값을 출력하면 다음과 같이 나온다.

```
user ~ $ rostopic echo /counter
WARNING: no messages received and simulated time is active.
Is /clock being published?
```
- 이유는 토픽에서 아무런 메시지를 받고 있지 않기 때문!

```
rostopic pub /counter std_msgs/Int32 5
```
다음과 같은 메시지를 주게 되면 echo 시 데이터가 정상적으로 출력되는 것을 확인할 수 있다.

---
## 사용자 지정 메시지 컴파일을 위한 준비

- 'rosmsg list'에 원하는 메시지 타입이 없을 경우 새로운 메시지를 제작할 수 있다.
- 메시지 제작 과정은 아래 순서를 따른다.
  1. 패키지 내에 msg 디렉토리를 생성
  2. 해당 디렉토리 내에 (메시지 이름).msg 파일을 생성
  3. CMakeLists.txt 파일을 수정
  4. package.xml 파일을 수정
  5. 컴파일 후 사용


### 1. Age.msg 작성

```
roscd <package_name>
mkdir msg
```
> Age.msg
```
float32 years
float32 months
float32 days
```
### 2. CMakeLists.txt 수정
- find_package()
```
find_package(catkin REQUIRED COMPONENTS
       roscpp
       std_msgs
       message_generation   # Add message_generation here, after the other packages
)
```

- add_message_files()
```
add_message_files(
  FILES
  Age.msg
)
```
- generate_messages()
```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
- catkin_package()
```
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES topic_subscriber_pkg
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
```

- package.xml 파일에서는 아래 내용을 추가해줘야함
```
<build_depend>message_generation</build_depend>

<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```