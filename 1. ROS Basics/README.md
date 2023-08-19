## 패키지를 컴파일하는 2가지 방법

- 패키지 생성 및 스크립트 작성 이후 이를 실행하기 위해 컴파일해야한다.
- 컴파일의 방법은 여러 종류가 있지만 해당 코스에서는 2가지만 소개함.



### catkin make
```
$ catkin_make
```
- workspace 디렉토리에서만 작동함(필수)
- 모든 src 디렉토리를 컴파일함

```
$ source devel/setup.bash
```
- 컴파일 후에는 다음 명령을 반드시 실행해주어야 한다.
- 최신 변경 사항을 적용하기 위함.

```
$ catkin_make --only-pkg-with-deps <package_name>
```
- workspace 내 모든 패키지를 컴파일하는 것이 아닌 특정 패키지만 컴파일하고 싶을 경우 다음과 같은 명령 사용

---


```
$ rospack profile
```
- ros가 때때로 새로운 패키지를 감지하지 못했을 경우 launch를 실행하지 못한다.
- 이때 새로고침해주는 명령어


---
## CMakeList.txt 수정하기
```
add_executable(simple src/simple.cpp)
add_dependencies(simple ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple
   ${catkin_LIBRARIES}
 )
```
- C++로 프로그램을 작성했을 경우 패키지 내의 CMakeList.txt 를 수정해야한다.
- 아래는 각각의 역할

```
add_executable(simple src/simple.cpp)
```
- C++파일의 실행 파일을 생성
- 실행 파일은 devel/lib에 있음

```
add_dependencies(simple ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```
- 실행 파일의 모든 의존성을 추가함

```
target_link_libraries(simple
   ${catkin_LIBRARIES}
 )
```
- 지정된 대상을 연결할 때 사용할 라이브러리 지정
- 위 줄에서는 생성한 실행 파일을 연결하기 위해 catkin 라이브러리를 사용한다.

아래 링크에서 CMakeLists에 대한 내용 확인 가능
http://wiki.ros.org/catkin/CMakeLists.txt

## 파라미터 서버
- 파라미터 서버는 파라미터를 저장하는 딕셔러니이다.
- 이 파라미터들은 런타임동안 사용되며 정적 데이터에 쓰인다.

```
$ rosparam list
$ rosparam get <parameter_name>
$ rosparam set <parameter_name> <value>
```

## 환경변수
- ROS는 적절한 작업 환경을 위해 리눅스의 환경변수를 사용한다.
- 아래 타이핑을 통해 이 환경변수를 확인할 수 있다.

```
$ export | grep ROS
```

```
declare -x ROSLISP_PACKAGE_DIRECTORIES="/home/user/catkin_ws/devel/share/common-lisp"
declare -x ROS_DISTRO="indigo"
declare -x ROS_ETC_DIR="/opt/ros/indigo/etc/ros"
declare -x ROS_MASTER_URI="http://localhost:11311"
declare -x ROS_PACKAGE_PATH="/home/user/catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks"
declare -x ROS_ROOT="/opt/ros/indigo/share/ros"```