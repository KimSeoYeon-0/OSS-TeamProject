목표: 상대편으로 공을 많이 보내기
*빨간색 밟으면 실격패
*상대팀의 로봇과 닿았을 경우 실격패
*상대편 경기장 하프라인의 표면(매트 및 경사로의 경사면)에 닿으면 실격패
*공이 경기장 밖으로 나갔을 시 공을 던진 팀의 절반 영역으로 반환

1. 로봇이 공을 인지하지 못했을 때 자유롭게 돌아다님
(1). 빨간색 라인 밖으로 나가지 않게 설정 (컬러 센서 사용) - 14주차 구현 완료
(2). 벽에 닿았을 때 or 로봇이 구석에 들어가버렸을 때 대비
-  초음파 센서 사용 (ex. 0cm 거리 안에 물체가 있다고 인지했을 시 후진) - 14주차 구현 완료 / tracking과 동시 적용이 가능하도록 알고리즘 수정 필요
(3). 처음 시작과 동시에 로봇이 회전을 하며 공을 찾음
2. 로봇이 공을 인지했을 때 (pd control) 공을 tracking 함
- 색상 threshold 설정 (경기장 색상 및 조명 고려) - 14주차 진행 완료
- 로봇 인지 범위 고려 (카메라 높이, 각도 등) - 14주차 구현 완료
3. 공을 잡고 슈팅 
- 공을 잡고 기다리는 시간, 쏠 때 속도 설정 - 14주차 진행 중 (큰 문제는 없으나 좀 더 수정하면 좋음)
- 공을 잡았을 시의 각도가 정면이 아닐 가능성이 높음 -> 자이로 센서 각도 초기화 후 슈팅 - 확인 필요
4. 자유롭게 돌아다님 (반복)

++ 경기장 환경에 맞춘 로봇 개조
- 컬러 센서 추가 (경사면 색상 인지)
- 초음파 센서 추가 (벽에 접근한 걸 판단)