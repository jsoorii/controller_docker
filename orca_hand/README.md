# Orca Hand

오픈소스 17-DOF 텐던 구동 로봇 손. 학술 및 산업용 조작 연구를 위해 설계된 저비용 고성능 손형 로봇.

- **웹사이트**: https://www.orcahand.com
- **논문**: arxiv:2504.04259 (IROS 2025 제출)
- **GitHub (소프트웨어)**: https://github.com/orcahand/orca_core
- **GitHub (URDF/MuJoCo)**: https://github.com/orcahand/orcahand_description

---

## 사양

| 항목 | 사양 |
|------|------|
| DOF | 17 (손가락 16 + 손목 1) |
| 구동 방식 | 텐던(tendon) 구동 |
| 모터 | DYNAMIXEL XC330-T288-T × 16 + XC430-T240BB-T × 1 |
| 총 비용 | ~2,000 CHF (약 300만원) |
| 조립 시간 | ~8시간 |
| 출판 | IROS 2025 |
| 특징 | 촉각 센서 내장, 좌/우 버전 |

### DOF 상세 구조

| 부위 | 조인트 | DOF |
|------|--------|-----|
| 손목 | wrist | 1 |
| 엄지 | cmc · abd · mcp · pip | 4 |
| 검지 | abd · mcp · pip | 3 |
| 중지 | abd · mcp · pip | 3 |
| 약지 | abd · mcp · pip | 3 |
| 소지 | abd · mcp · pip | 3 |
| **합계** | | **17** |

> DIP(원위지간) 관절은 텐던 연동으로 PIP와 연결 — 독립 액추에이터 없음

### 액추에이터 제어 범위 (v2 MuJoCo 기준, 단위: rad)

| 액추에이터 | 최소 | 최대 |
|-----------|------|------|
| wrist | -1.13 | 0.61 |
| finger abd (검지/중지/약지/소지) | -0.52 | 0.52 |
| finger mcp | -0.44 | 1.75 |
| finger pip | -0.26 | 1.87 |
| thumb cmc | -0.79 | 0.58 |
| thumb abd | -0.31 | 0.96 |
| thumb mcp | -0.44 | 1.75 |
| thumb pip | -0.26 | 1.87 |

---

## 파일 구조

```
orca_hand/
├── README.md               ← 이 파일
├── bom.csv                 ← 부품 목록 (BOM)
├── ORCA_v1.step            ← 전체 어셈블리 STEP 파일 (41MB)
├── orcahand_description/   ← 공식 GitHub 레포 클론 (URDF + MuJoCo)
│   ├── v1/                 ← 레거시 버전 (legacy)
│   │   ├── scene_left.xml / scene_right.xml / scene_combined.xml
│   │   ├── scene_*_extended.xml  (카메라·U2D2·팬 포함)
│   │   └── models/urdf/ + models/mjcf/
│   └── v2/                 ← 최신 버전 (Fusion360 export, 권장)
│       ├── scene_left.xml / scene_right.xml / scene_combined.xml
│       └── models/urdf/ + models/mjcf/
├── stl/                    ← STL 카테고리별 ZIP
│   ├── ORCA_Fingers.zip        (손가락 파트 16개 STL)
│   ├── ORCA_Misc.zip           (기타 파트)
│   ├── ORCA_Molds.zip          (실리콘 패드 몰드)
│   ├── ORCA_Spools.zip         (텐던 스풀)
│   └── ORCA_Tower.zip          (모터 타워/팜 구조체)
└── bambu/                  ← Bambu 3D 프린터용 .3mf 파일
    ├── ORCA_CarpalsWrist-Left.3mf
    ├── ORCA_CarpalsWrist-Right.3mf
    ├── ORCA_Fingers-Left.3mf
    ├── ORCA_Fingers-Right.3mf
    ├── ORCA_Letters.3mf
    ├── ORCA_Molds-Left.3mf
    ├── ORCA_Molds-Right.3mf
    ├── ORCA_Spools.3mf
    └── ORCA_Tower.3mf
```

### MuJoCo 시뮬레이션 실행 방법

```bash
cd orca_hand/orcahand_description
pip install mujoco

# v1 (레거시)
python -m mujoco.viewer --mjcf=$(pwd)/v1/scene_combined.xml

# v2 (최신, 권장)
python -m mujoco.viewer --mjcf=$(pwd)/v2/scene_combined.xml
```

### 버전 차이 (v1 vs v2)

| 항목 | v1 | v2 |
|------|----|----|
| 출처 | 수동 작성 | Fusion360 자동 export |
| URDF 링크 수 | 42 (offset body 포함) | 19 (간결) |
| extended 버전 | 있음 (카메라·U2D2·팬) | 없음 |
| 권장 여부 | 레거시 | 권장 |

---

## STL 파일 목록

### ORCA_Fingers (손가락 파트)
| 파일명 | 설명 |
|--------|------|
| DP.stl | 원위지골 (Distal Phalanx) |
| L-AP.stl | 좌측 손바닥 (Abductor Plate) |
| L-I-PP.stl | 좌측 검지 근위지골 |
| L-IP.stl | 좌측 중간지골 (Intermediate Phalanx) |
| L-M-PP.stl | 좌측 중지 근위지골 |
| L-P-PP.stl | 좌측 소지 근위지골 |
| L-T-AP.stl | 좌측 엄지 손바닥 |
| L-T-DP.stl | 좌측 엄지 원위지골 |
| R-AP.stl | 우측 손바닥 |
| R-I-PP.stl | 우측 검지 근위지골 |
| R-IP.stl | 우측 중간지골 |
| R-M-PP.stl | 우측 중지 근위지골 |
| R-P-PP.stl | 우측 소지 근위지골 |
| R-T-AP.stl | 우측 엄지 손바닥 |
| R-T-DP.stl | 우측 엄지 원위지골 |
| R-T-PP.stl | 우측 엄지 근위지골 |

### ORCA_Tower (모터 타워/팜 구조)
총 17개 STL 파일 (모터 마운트, 팜 구조, 손목 연결부 등)

### ORCA_Molds (실리콘 패드 몰드)
총 14개 STL 파일 (촉각 센서용 실리콘 패드 제작 몰드)

### ORCA_Spools (텐던 스풀)
총 4개 STL 파일 (텐던 와이어 스풀)

### ORCA_Misc (기타)
1개 STL 파일

---

## BOM 요약 (부품 목록)

전체 BOM은 `bom.csv` 파일 참조.

### 전자부품 (Electronics)

| 부품 | 수량 | 단가(CHF) | 합계(CHF) |
|------|------|-----------|-----------|
| DYNAMIXEL XC330-T288-T | 16 | 113 | 1,808 |
| DYNAMIXEL XC430-T240BB-T | 1 | 123 | 123 |
| U2D2 Controller | 1 | 47.7 | 47.7 |
| U2D2 Power Hub | 1 | 34 | 34 |
| Power Supply 12V 5A | 1 | 39.3 | 39.3 |
| USB Cable | 1 | 11.85 | 11.85 |
| Motor Connectors | 13 | 11 | 143 |
| Cooling Fan 12V 50x50x10 | 2 | 8.61 | 8.61 |

### 기계부품 (Mechanical)

| 부품 | 수량 | 합계(CHF) |
|------|------|-----------|
| Bearing 4x8x3 | 35 | 35.28 |
| Bearing 6x13x5 | 2 | 6.5 |
| Magnets 5x2.8 | 10 | 4.4 |
| Tendon 0.39-0.41mm | 15m | 176 |
| Pin 2x6 | 20 | 5 |
| PTFE Tube 0.9x1.5 | 1 | 13 |
| GT2 Belt 140 | 1 | 8.84 |
| M4, M2, M1.4 스크류/너트/와셔 세트 | - | - |
| Rod 3x55 | 4 | 23.65 |

### 재료 (Material)

| 부품 | 수량 | 합계(CHF) |
|------|------|-----------|
| PLA Filament Black | 1 roll | 15 |
| Silicone Dragon Skin 10NV | 1 | 44.9 |

**총 예상 비용: ~2,000 CHF (전자부품 포함)**

---

## 논문 정보

- **제목**: ORCA Hand: An Open-Source Dexterous Robotic Hand
- **저자**: ETH Zurich 연구팀
- **논문**: https://arxiv.org/abs/2504.04259
- **학회**: IROS 2025

### 주요 특징
- 17-DOF 텐던 구동 (손가락 16 + 손목 1), DIP는 PIP와 텐던 연동
- 모든 파트 3D 프린팅 가능 (PLA), Bambu 프린터용 .3mf 제공
- 실리�on 촉각 패드 내장 (Dragon Skin 10NV 몰드)
- DYNAMIXEL 모터 사용 → ROS2 호환
- 좌/우 대칭 설계, MJCF에서 position actuator로 제어 추상화
- URDF + MuJoCo(MJCF) 모델 모두 제공, v1(레거시)/v2(최신) 두 버전

---

## 관련 링크

| 자료 | URL |
|------|-----|
| 공식 사이트 | https://www.orcahand.com |
| 파일 다운로드 페이지 | https://www.orcahand.com/legacy/files |
| orca_core (소프트웨어) | https://github.com/orcahand/orca_core |
| orcahand_description (URDF/MuJoCo) | https://github.com/orcahand/orcahand_description |
| 논문 (arxiv) | https://arxiv.org/abs/2504.04259 |
| ROBOTIS (모터 구매) | https://www.robotis.us |
