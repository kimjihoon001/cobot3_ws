# Rokey - A-2 (cobot3_ws)

ROKEY 협동3기 팀 프로젝트 워크스페이스입니다.
ROS2(`src/`) 와 Isaac Sim 스크립트(`isaacpjt/`) 를 하나의 Git 저장소로 관리합니다.

> **목표:** 공용 GPU 노트북 환경에서 USB·메신저 없이 협업한다.
> **원칙:** `git pull` 한 번으로 팀원들의 최신 폴더와 파일을 그대로 가져올 수 있어야 한다.

---

## 📁 저장소 구조

```
~/cobot3_ws/                     # 저장소 루트
├── src/                         # ROS2 패키지
│   └── m0609/                   # 두산 M0609 매니퓰레이터 작업
│       ├── hyeonminlee/         # ← 개인 폴더
│       ├── jihoonkim/           # ← 개인 폴더
│       └── minseongkim/         # ← 개인 폴더
├── isaacpjt/                    # Isaac Sim 스크립트
│   └── basic/
│       ├── hyeonminlee/         # ← 개인 폴더
│       ├── jihoonkim/           # ← 개인 폴더
│       └── minseongkim/         # ← 개인 폴더
├── build/  install/  log/       # colcon 산출물 → .gitignore 제외
├── .gitignore
└── README.md
```

- **이론 기간:** 각자 자기 이름 폴더에서만 작업 → 충돌이 거의 발생하지 않음
- **프로젝트 기간:** 역할별 폴더로 재편성 예정 (예: `manipulator_ctrl/`, `amr_ctrl/`)

---

## 👥 역할

| 역할 | 담당 | 책임 |
|------|------|------|
| 팀장 | hyeonminlee | 원격 저장소 생성·관리, 멤버 권한 설정, `main` 브랜치 merge |
| 팀원 | jihoonkim, minseongkim | clone, 개인 폴더 작업, add / commit / push |

원격 저장소: <https://github.com/kimjihoon001/cobot3_ws.git>

---

## 🔧 사전 준비 (최초 1회)

1. **Git 설치** — GPU 노트북과 개인 PC 모두
2. **VSCode + Git 확장** 설치
3. **GitHub 계정** 생성 후 팀장에게 collaborator 등록 요청
4. **PAT(Personal Access Token) 발급** — HTTPS로 push 할 때 비밀번호 대신 사용
   - GitHub → Settings → Developer settings → Personal access tokens → 발급
   - `repo` 권한 체크, 만료일 설정 후 토큰 복사 (재확인 불가하니 보관 주의)

### 사용자 정보 설정

```bash
git config --global user.name "본인이름"
git config --global user.email "본인GitHub이메일"
```

### Roboflow 학습 데이터 다운로드

API 키가 저장소와 셸 히스토리에 남지 않도록 환경변수로 설정합니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r yolo_training/requirements.txt
cp .env.example .env
# .env를 열어 ROBOFLOW_API_KEY 값을 발급받은 실제 키로 변경

# 1순위 Fresh/Rotten 베이스 데이터셋만 다운로드
python yolo_training/scripts/download_roboflow_datasets.py

# 보강용 12클래스 데이터셋만 다운로드
python yolo_training/scripts/download_roboflow_datasets.py --dataset booster

# 두 데이터셋 모두 다운로드
python yolo_training/scripts/download_roboflow_datasets.py --dataset all
```

다운로드 결과는 Git에서 제외된 `yolo_training/raw/` 아래에 저장됩니다. 원본은
직접 수정하지 않고 이후 전처리 결과를 별도 폴더에 생성합니다.

Ubuntu/Debian이 관리하는 시스템 Python에는 `--break-system-packages`로 설치하지
않습니다. 터미널을 새로 열었다면 `source .venv/bin/activate`를 다시 실행합니다.

### 토마토 데이터 전처리

다운로드 원본은 유지하면서 학습용 bbox 데이터 두 종류를 생성합니다.

```bash
python yolo_training/scripts/preprocess_tomato_dataset.py
```

- `yolo_training/processed/tomato_detection`: 모든 토마토를 `tomato` 1클래스로 통합
- `yolo_training/processed/tomato_quality`: Fresh/Rotten 2클래스를 유지
- `yolo_training/reports/preprocess_summary.json`: 변환 통계와 제외된 라벨 기록

### YOLO 2단계 학습

```bash
# Quality train split을 Fresh/Rotten 균형에 가깝게 재구성
python yolo_training/scripts/create_balanced_quality_dataset.py

# 1단계: 균형화한 공개 실사진으로 Fresh/Rotten Detection 학습
python yolo_training/yolo_train.py --device 0

# 2단계: 1단계 best.pt를 자동 선택해 Isaac Sim 캡처 데이터로 파인튜닝
python yolo_training/yolo_finetune_own.py --data yolo_training/simulation/data.yaml --device 0
```

시뮬레이션 데이터의 클래스는 1차 모델과 동일한 `Fresh Tomato`, `Rotten Tomato`
2클래스여야 하며, 결과와 최종 `best.pt`는 `yolo_training/runs/` 아래에 저장됩니다.

### 저장소 clone

```bash
cd ~
git clone https://github.com/kimjihoon001/cobot3_ws.git
cd cobot3_ws
```

> push 시 아이디/비밀번호를 물으면 비밀번호 자리에 **PAT** 를 붙여넣습니다.

---

## 🔄 매일 하는 작업 흐름

> **작업 시작 전 반드시 `git pull` → 끝나면 `add · commit · push`**

```bash
cd ~/cobot3_ws
git pull                         # ① 최신 상태로 동기화 (제일 먼저!)

# ② 내 이름 폴더에서 작업 수행

git add .                        # ③ 변경분 스테이징
git commit -m "작업 내용 요약"    # ④ 커밋
git push                         # ⑤ 원격에 업로드
```

상태 확인이 필요하면:

```bash
git status        # 변경된 파일 확인
git log --oneline # 커밋 히스토리 확인
```

---

## ✅ 협업 규칙

- **내 이름 폴더만 수정한다.** 다른 사람 폴더는 절대 건드리지 않는다.
- **작업 전 `git pull` 을 습관화한다.** (충돌 예방의 핵심)
- **`main` merge 는 팀장이 한다.** 팀원은 임의로 merge 하지 않는다.
- **커밋은 자주, 의미 단위로.** "하루치 몰아서" 대신 작은 단위로 나눠 커밋한다.
- **커밋 메시지는 한국어로 명확하게.** 무엇을 왜 바꿨는지 알 수 있게 작성한다.

### 커밋 메시지 예시

```
매니퓰레이터 픽앤플레이스 기본 동작 구현
RGB 카메라 모니터 노드 추가
gripper open/close 파라미터 조정
```

---

## 🚫 .gitignore (커밋하지 않는 것)

빌드 산출물·대용량 파일·머신별 설정은 저장소에 올리지 않습니다.

- `build/`, `install/`, `log/` — colcon 빌드 산출물
- `__pycache__/`, `*.pyc` — 파이썬 캐시
- `.vscode/`, `.claude/` — 머신마다 다른 로컬 설정
- `*.usd`, `*.usda`, `*.usdc` — USD (스크립트로 재생성)
- `*.onnx`, `*.pt`, `*.pth` — 학습 가중치
- 개인 메모 문서 (`CLAUDE.md`, `*_NOTES.txt` 등)

> 빈 폴더는 Git이 추적하지 않으므로, 자리를 유지하려면 폴더 안에 `.gitkeep` 파일을 둡니다.

---

## 🆘 자주 겪는 문제

| 상황 | 해결 |
|------|------|
| `git push` 가 거부됨 (rejected) | 원격에 새 커밋이 있음 → `git pull` 후 다시 push |
| 비밀번호를 물어봄 | PAT 를 비밀번호 자리에 입력 |
| 충돌(conflict) 발생 | 표시된 파일을 열어 `<<<<<<<` 부분을 정리 → `add` → `commit`. 애매하면 팀장과 상의 |
| 실수로 다른 폴더 수정 | 커밋 전이면 `git checkout -- <파일>` 로 되돌리기 |
| `colcon build` 시 `smartfarm_interfaces`에서 `ModuleNotFoundError: No module named 'em'` / `'catkin_pkg'` | `./scripts/fix_ros_build_env.sh` 실행 — `$ROS_DISTRO`를 감지해서 `.venv`에 `catkin_pkg`/`lark`/맞는 버전의 `empy`를 자동으로 설치한다(Humble=3.3.4 고정, Jazzy=4.x). 배포판별 버전을 섞어 쓰면 다른 에러로 빌드가 깨지므로 수동으로 `pip install empy`만 하지 말 것 |

---

*문의: 팀 채널에서 팀장(hyeonminlee)에게*
