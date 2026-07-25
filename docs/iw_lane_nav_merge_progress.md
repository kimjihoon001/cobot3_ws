# IW lane-nav 병합 진행·인수인계 기록

- 작업일: 2026-07-25
- 대상 브랜치: `harvest/rmp-mm-suction`
- 병합 대상: `iw-lane-nav` (`55263ea`)
- 상태: `git merge --no-commit --no-ff iw-lane-nav` 실행 후 충돌 검토 중
- 주의: `src/m-explore-ros2/`는 미추적 외부 작업이므로 병합 커밋에 포함하지 않는다.

이 문서는 병합 도중 작업자가 바뀌어도 확정된 선택, 완료된 수정, 남은 검증을
그대로 이어가기 위한 체크포인트다.

## 1. 확정된 병합 기준

1. KLT 충돌체: `iw-lane-nav`의 명시적 열린 상자 shell 사용
2. IW 경로: `iw-lane-nav`의 레인 기반 `NavigateThroughPoses` 사용
3. FOLLOW 위치: 현재 브랜치의 IW 접근 방향 기반 `dock_standoff=1.2m` 유지
4. 미션:
   - `IDLE`, `FOLLOW`: 현재 브랜치 의미 유지
   - `FORKLIFT`: `iw-lane-nav`의 전체 팔레트 교환 상태 머신 사용
   - `LOAD`: 최종 미션 목록에서 제외
5. 컨트롤러: 레인·swept-footprint + DWB 절충안
   - `min_vel_x=0.0`으로 일반 주행 후진 금지
6. 로컬 inflation: `iw-lane-nav` 값
   - `cost_scaling_factor=6.0`
   - `inflation_radius=0.55`
7. map→odom: 현재 브랜치의 AMCL 동적 보정 유지
8. 라이다 자기반사 필터: 현재 브랜치 유지
9. MM Nav2 DWB: 현재 브랜치 설정 유지
10. 창고 단독 실행 판정: `iw-lane-nav` 조건 사용

최종 주행 조합:

```text
AMCL 동적 map→odom
        ↓
레인 기반 NavigateThroughPoses
        ↓
swept-footprint 사전 검사
        ↓
DWB (min_vel_x=0.0)
```

## 2. 구현 시 반드시 함께 맞출 부분

### 미션 호환성

병합 대상의 수확 데모가 플레이스 전에 `/iw/mission=LOAD`를 보내고
`/iw/status=READY_LOAD`를 기다리던 흐름은 제거했다. 확정안대로 FOLLOW의 1.2m
standoff 상태에서 KLT 도달성을 확인하고 바로 플레이스한다.

### TF 단일 발행자

- AMCL: `tf_broadcast=true`
- 고정 `map→odom` static publisher: 실행하지 않음
- mission node의 고정 초기 map→odom fallback: 사용하지 않음
- 실제 AMCL의 `map→odom` TF를 받은 뒤에만 IW map pose와 레인 경로를 생성

### DWB와 레인 경로

- RPP 플러그인과 RPP 전용 파라미터를 남기지 않는다.
- DWB critic을 유지한다.
- `min_vel_x=0.0`으로 일반 주행 중 임의 후진을 금지한다.
- 전진 전용 BT, 첫 웨이포인트 방향 정렬 및 도크 최종 저속 정렬의 호환성을 확인한다.

## 3. 단계별 진행 상황

- [x] 양쪽 브랜치와 기준 문서 비교
- [x] 충돌 파일 6개 식별
- [x] 사용자 병합 기준 확정
- [x] KLT shell과 창고 단독 실행 판정 충돌 해소
- [x] IDLE/FOLLOW + forklift 상태 머신 통합
- [x] LOAD/READY_LOAD 호출 제거 또는 FOLLOW 흐름으로 복원
- [x] 레인 X 스냅으로 1.2m standoff가 소실되지 않도록 정확한 최종점 보존
- [x] swept-footprint를 현재 Nav2 적재 외곽(+0.65/-1.08, 폭 0.802m)과 통일
- [x] 통합 launch의 과거 follow_offset 인자를 dock_standoff=1.2로 정리
- [x] FOLLOW goal tolerance 0.25m/0.25rad 유지, forklift 정밀도는 별도 폐루프로 분리
- [x] 레인·swept-footprint + DWB 통합
- [x] inflation 0.55/6.0 적용
- [x] AMCL 동적 TF + scan self-filter 유지
- [x] MM Nav2 설정 현재 브랜치 유지
- [x] 충돌 마커 및 미해결 인덱스 0개 확인
- [x] 정적 검사·단위 테스트
- [ ] Isaac Sim 실제 통합 검증
- [x] 병합 커밋 `58ee69b`

## 4. 검증 체크리스트

### 정적·단위 테스트

- [x] 변경 Python 파일 `py_compile`
- [x] deck geometry 핵심 5개 케이스 통과
- [x] 레인 경로·확장 footprint·충돌 거부 핵심 케이스 통과
- [x] IW deadband 핵심 케이스 통과
- [x] 관련 ROS 패키지 6개 `/tmp` 격리 빌드 통과
- [x] `git diff --check` 통과
- [x] RPP 파라미터가 DWB 설정에 섞이지 않았는지 검색
- [x] `src/m-explore-ros2/`가 staged 목록에 없는지 확인

환경에 `pytest` 실행 파일이 없어 pytest 전체 수집은 실행하지 못했다. 대신 해당
테스트의 핵심 입력을 직접 실행했고, ROS 패키지 빌드까지 통과했다. 빌드에서는 기존
setuptools의 `tests_require` 경고만 있었고 실패는 없었다.

### ROS/Isaac 통합 검증

- [ ] 시작 상태가 IDLE이며 명령 전 IW가 움직이지 않음
- [ ] FOLLOW가 현재 IW 접근 방향 기준 1.2m 목표를 생성
- [ ] 최종 1.2m 접근 구간이 배드를 가로지르면 goal을 보내지 않고 안전 정지
- [ ] FOLLOW 경로가 레인 중심과 swept-footprint 검사를 통과
- [ ] DWB가 전진·곡선·제자리 회전을 수행하고 임의 후진하지 않음
- [ ] AMCL만 `map→odom`을 발행하며 TF 중복·점프가 없음
- [ ] 전·후방 filtered scan에 IW 자체 팔레트/KLT 반사가 제거됨
- [ ] 8개 KLT 모두 바닥·벽 충돌체가 있고 토마토가 관통하지 않음
- [ ] FORKLIFT가 접근→정렬→서비스→clear 대기→MM 복귀까지 수행
- [ ] 도킹 재정렬 요청과 최대 재시도 제한이 동작
- [ ] RETURNED 및 `/iw/resume_harvest=True` 후 FOLLOW 재개

## 5. 커밋 이후 남은 작업

충돌 해소안은 `58ee69b`로 병합 커밋했다. 아직 Isaac Sim 실제 통합 검증은 하지
않았으므로 위 체크리스트의 ROS/Isaac 항목은 다음 작업자가 실행 결과와 함께
갱신해야 한다.
