# GENE-SKU6 SBC, Xenomai 3.1 + SOEM

## 사용법

터미널 3개를 연 뒤, 다음 2개의 명령어를 각 터미널에 입력합니다.

```bash
$ roslaunch elmo_ethercat main.launch

$ rosrun elmo_ethercat control_maxon44
```

main 코드에서 Actual Position을 알 수 있습니다.   

control_maxon 코드에서 키보드 입력을 통해, 특정 Position으로 모터를 동작시킬 수 있습니다.
