# 프로젝트 구조

> 출처: Notion 제어기 설계 > 프로젝트 구조 (2024-05-26)

```
proj/
├── comm/                          # 통신 모듈
│
├── control/                       # 제어 알고리즘 레이어
│   ├── core/                      # 메인 제어 엔진 및 파서
│   │   ├── grasp_controller       # 파지 제어기
│   │   └── control_parser         # 제어 명령 파서
│   ├── kinematics/                # 기구학 상위 로직
│   ├── linkage/                   # 하드웨어 특화 링크 구현
│   ├── math/                      # 수치 연산 유틸
│   └── util/                      # 공통 유틸리티
│
├── core/                          # 시스템 초기화 및 엔트리 포인트
│   └── inc/                       # 헤더
│       └── hand memory define, function
│
└── drivers/                       # 드라이버 레이어
    └── custom/                    # 사용자 하드웨어 드라이버
        └── inc/                   # 헤더 (motor, sensor comm)
```

## 모듈 설명

| 모듈 | 경로 | 역할 |
|------|------|------|
| 통신 | `proj/comm/` | 외부 인터페이스 통신 모듈 |
| 제어 엔진 | `proj/control/core/` | 메인 제어 루프, 명령 파서 |
| 파지 제어기 | `proj/control/core/grasp_controller` | GraspController — 상태머신, RT 루프 |
| 제어 파서 | `proj/control/core/control_parser` | 수신 명령 파싱 및 분배 |
| 기구학 | `proj/control/kinematics/` | 손가락 FK/IK, 자코비안 상위 로직 |
| 링크 구현 | `proj/control/linkage/` | 4절 링크 등 하드웨어 특화 기구학 |
| 수학 | `proj/control/math/` | 행렬, 수치 미분 등 수치 연산 |
| 유틸 | `proj/control/util/` | 공통 헬퍼 함수 |
| 시스템 초기화 | `proj/core/` | 부팅, 엔트리 포인트, 메모리 정의 |
| 하드웨어 드라이버 | `proj/drivers/custom/` | 모터·센서 통신 드라이버 헤더 |
