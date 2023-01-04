# 3D pose Action Data Master

## Introduction
멀티 Slave 컴퓨터의  키넥트를 통해 3차원 자세 Action 데이터를 추출하기 위한 마스터 트리거 코드 


## Prerequisites
- Python3 (over 3.9) cause of multiprocessing shared memory
- Other Python libraries can be installed by `pip install -r requirements.txt`


## How to use

```shell
# with realtime pose estimation
Action_Data_Saver.py 내 변수
HOST = '192.168.0.x'#Master 컴퓨터 ip

CLIENTS_LIST = ['192.168.0.a', '192.168.0.b'] #slave 컴퓨터 IP EX) 3개 컴퓨터 이용시 [A, B, C]
Port = [5555, 7777] #slave 컴퓨터마다 통신할 port 지정 EX) 3개 컴퓨터 이용시 [가, 나, 다]
num_com = 1 #slave 컴퓨터개수
num_source = 2 #한 slave 컴퓨터에 연결된 kinect 개수
fps_cons = 30 #fps제한 (컴퓨터에 따라 다를 수 있습니다. 적절히 조절 slave 컴퓨터의 kinect fps와 같도록)
```
```shell
#Slave 컴퓨터의 pyKinectAzure의 example_for_save_data.py를 전부 실행
python Action_Data_Saver.py #코드 실행
#1 subject 숫자 입력
#2 action 번호 입력
#3 Slave 컴퓨터 모든 kinect가 준비될 때까지 대기 
#4 a를 눌러 데이터 수집 시작
#5 q를 눌러 데이터 수집 종료
#1 반복
```




## Contact
For any question, feel free to contact
```
이혁상    : melungl@yonsei.ac.kr
```
