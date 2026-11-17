# 이전 장노출 사진으로 병렬 보정

현재 기본 흐름:

1. MANUAL에서 스위치 클릭 → 최초 사진 촬영 및 plate solving → 추적 목표 고정.
2. 10초씩 3장을 순서대로 촬영하고 노출 중 목표 RA/Dec를 추적.
3. 두 번째 노출 중 첫 사진을 풀어 보정. 그 풀이가 끝났으면 세 번째 노출 중 두 번째 사진을 풀어 보정.
4. 사진 3장을 저장하고 진행 중인 풀이가 끝나면 모터를 정지한 뒤 프로그램 종료.

`config.py` 설정:

```python
PIPELINED_CAPTURE_ENABLED = True
PIPELINED_EXPOSURE_SECONDS = 10.0
PIPELINED_FRAME_COUNT = 3
PIPELINED_SKIP_FINAL_CORRECTION = False
EXIT_AFTER_CAPTURE = True
```

사진은 실행 디렉터리 기준 `data/images/final_sequence/final_<시각>_<001~003>.jpg`에 개별 저장된다.
초기 목표 풀이와 사진 저장 시간은 총 노출 30초와 별도다.
실제 노출 시간은 카메라 메타데이터에서 확인할 수 있다.

한 번에 한 장만 푼다. 첫 사진의 풀이가 진행 중이면 세 번째 노출은 바로 시작하고 두 번째 사진 풀이는 건너뛴다.
`CORRECTION_INTERVAL=120`은 주기 보정 모드에 적용된다.

카메라 메타데이터로 노출 중간 시각을 구하고 GNSS UTC에 맞춰 오차를 계산한다.
이전 사진의 절대 Alt/Az로 현재 모터 위치를 덮어쓰지 않는다.
촬영 시작 이후 보정이 진행된 프레임은 평균 방향을 보여주므로 정확도 한계가 있다.

검증: 하드웨어 없는 자동 테스트에서 분석/촬영 동시 실행, 마지막 보정 생략,
중단 후 늦은 보정 차단, 저장 실패 시 기존 경로 보존, 완료 후 수동 복귀 생략을 확인.
실제 Picamera2 센서 노출 시간, 장노출 결과 및 모터 보정은 현장 검증 대상이다.
