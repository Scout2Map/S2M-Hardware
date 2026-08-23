# S2M-Hardware

Scout2Map UGV의 하드웨어 설계 자료를 관리하는 저장소다. 전자 부품 및 회로 정보는 [`electronics`](./electronics) 폴더에, 차체 설계 관련 자료는 [`mechanical`](./mechanical) 폴더에, ROS2/Gazebo 시뮬레이션용 URDF-xacro 모델은 [`UGV_description`](./UGV_description) 폴더에 위치한다.

## 폴더 구조

```
S2M-Hardware/
├── electronics/            # 전자 부품 스펙, 회로도, 핀맵 등
├── mechanical/              # 차체 설계 도면, 가공 파일 등
├── UGV_description/         # ROS2 패키지: URDF-xacro 모델 + 시뮬레이션 에셋
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── urdf/
│   │   └── scout2map.urdf.xacro   # 로봇 모델 본체
│   ├── config/
│   │   └── bridge.yaml            # gz ↔ ROS2 토픽 브릿지 설정
│   ├── worlds/
│   │   ├── slip_test.world.sdf    # 요철·슬립 테스트용 월드
│   │   └── bump_field.png         # 요철 지형용 heightmap 이미지
│   └── launch/
│       ├── spawn_robot.launch.py       # 로봇 스폰 + robot_state_publisher + 브릿지
│       └── slip_test_world.launch.py   # slip_test 월드 실행
└── README.md
```

`UGV_description`은 실제 차체 치수를 기반으로 한 URDF-xacro 모델과, 이를 Gazebo에서 바로 돌려볼 수 있는 launch/world 파일 일체를 담은 독립 ROS2 패키지다. 패키지 디렉터리명은 `UGV_description`이지만 `package.xml` 상의 ROS2 패키지 이름은 `s2m_description`이다 (colcon은 디렉터리명과 패키지명이 달라도 정상 빌드된다).

---

## 사용법 (UGV_description)

### 1. 워크스페이스에 배치 및 빌드

`UGV_description` 폴더를 ROS2 워크스페이스의 `src` 아래로 심볼릭 링크하거나 복사한 뒤 빌드한다.

```bash
ln -s ~/S2M-Hardware/UGV_description ~/scout_sim_ws/src/UGV_description
cd ~/scout_sim_ws
colcon build --packages-select s2m_description
source install/setup.bash
```

### 2. 요철·슬립 테스트 월드 실행

```bash
ros2 launch s2m_description slip_test_world.launch.py
```

Gazebo가 뜨면 요철 지형(heightmap)과 저마찰 슬립 구역(파란 반투명 박스, x=3m 지점)이 보인다.

### 3. 로봇 스폰

월드가 뜬 상태에서 별도 터미널을 열어 로봇을 스폰한다. `robot_state_publisher`, `ros_gz_bridge`가 함께 실행되며 `/scan`, `/imu`, `/odom`, `/tf`, `/cmd_vel` 토픽이 ROS2 쪽으로 연결된다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/scout_sim_ws/install/setup.bash
ros2 launch s2m_description spawn_robot.launch.py
```

필요 시 스폰 위치를 인자로 지정할 수 있다. `z_pose`(기본 0.05m)는 지면 간격(20mm)보다 살짝 높은 곳에서 떨어뜨려 물리엔진이 바퀴를 지면에 자연스럽게 안착시키기 위한 값이다.

```bash
ros2 launch s2m_description spawn_robot.launch.py x_pose:=0.5 y_pose:=0.0 z_pose:=0.05
```

### 4. 토픽 확인

```bash
ros2 topic list | grep -E "/(scan|imu|odom|cmd_vel|tf)"
ros2 topic echo /scan --once | head -20
```

이 시점부터는 기존 `scout_sim_bringup`에서 쓰던 것과 동일한 `slam_toolbox` / `teleop_twist_keyboard` 검증 절차를 그대로 적용할 수 있다. `/cmd_vel`로 주행하면서 요철 구간과 슬립 구역 통과 시 `/odom` 대비 실제 이동량 괴리를 관찰하는 방식으로 슬립·요철 이벤트 판별 로직을 검증한다.

> `gz-sim-diff-drive-system` 플러그인 이름은 gz-sim 버전에 따라 다를 수 있다. 로봇이 스폰됐는데 `/cmd_vel`에 반응하지 않으면 `gz sim --plugin-path`로 실제 플러그인 파일명을 확인해 `urdf/scout2map.urdf.xacro`의 `<plugin filename=...>` 값을 맞춰야 한다.

---

## 좌표계 정의

모든 치수는 `base_link` 원점을 기준으로 한다. `base_link` 원점은 **차체 플랫폼 바닥면의 중앙**으로 정의한다.

- x축: 전진 방향 (+)
- y축: 좌측 방향 (+)
- z축: 상방 (+)

지면 기준을 먼저 고정한다: **차체 바닥면(`base_link` 원점)은 지면에서 20mm 띄워진다 (`ground_clearance`).** 즉 지면은 항상 `base_link` 기준 z = −20mm이고, 바퀴는 반드시 이 지면에 접지해야 한다. 바퀴 축 높이는 여기서 거꾸로 계산한다 — `wheel_radius − ground_clearance`(현재 33mm − 20mm = +13mm)로, 축이 차체 바닥면보다 살짝 위(브라켓 안쪽)에 물리고 바퀴가 그만큼 아래로 튀어나와 지면에 닿는 구조다. 타이어를 바꿔 `wheel_radius`가 달라져도 `ground_clearance`는 그대로 두고 축 높이만 자동으로 다시 계산되도록 xacro에 수식으로 넣어뒀다.

> 이전 버전은 "브라켓 낙차 35mm"라는 고정값만 있고 실제 지면 위치에 대한 기준이 없어서, 계산해보면 지면이 `base_link` 기준 z = −68mm에 있는 셈이었다(축 −35mm − 반지름 33mm). RViz 그리드는 보통 오도메트리 시작점(≈`base_link` z=0)을 지면으로 그리기 때문에, 바퀴가 그 그리드보다 68mm 아래로 파고들어가 보이는 원인이었다. `ground_clearance` 기준으로 바꾸면서 이 불일치를 없앴다.

---

## 차체 본체 (base_link)

| 항목 | 값 |
|---|---|
| 크기 (L × W × H) | 265mm × 220mm × 150mm |
| 지면 간격 (ground clearance, 바닥면 → 지면) | 20mm |
| 지면 기준 전체 높이 (지면 → 차체 상단) | 170mm (20 + 150) |
| 질량 | 1800g |
| 무게중심 오프셋 (x, y, z) | (0, 0, 75mm) — 기하학적 중심 근사치, 배터리/SBC 배치 확정 시 보정 필요 |
| 관성 텐서 | 아래 표 참조 (박스 근사) |

### 관성 텐서 (box 근사, 무게중심 기준)

| 성분 | 값 (kg·m²) |
|---|---|
| I_xx | 0.01064 |
| I_yy | 0.01391 |
| I_zz | 0.0178 |
| I_xy, I_xz, I_yz | 0 (좌우/전후 대칭 가정) |

> H가 160mm→150mm로 바뀌면서 높이에 의존하는 I_xx, I_yy만 재계산했다. I_zz는 L, W에만 의존해서 그대로다.

---

## 바퀴 (wheel_link)

| 항목 | 값 |
|---|---|
| 바퀴 지름 | 66mm (반지름 33mm) |
| 바퀴 폭 | 26mm |
| 바퀴 질량 | 개당 35g |
| 트랙폭 (좌우 바퀴 중심 간 거리) | 240mm |
| 휠베이스 (전후 바퀴 중심 간 거리) | 140mm |
| 지면 간격 (ground clearance, 플랫폼 바닥면 → 지면) | 20mm |
| 바퀴 축 높이 (플랫폼 바닥면 → 바퀴 축 중심, = 반지름 − 지면 간격) | +13mm |

### 바퀴 장착 위치 (base_link 기준, 4륜 스키드 스티어)

| 바퀴 | x | y | z |
|---|---|---|---|
| front_left | +70mm | +120mm | +13mm |
| front_right | +70mm | −120mm | +13mm |
| rear_left | −70mm | +120mm | +13mm |
| rear_right | −70mm | −120mm | +13mm |

> 참고: 지면(바퀴 접지면)은 `base_link` 기준 z = −20mm(= −지면 간격) 위치이며, 이는 바퀴 반지름과 무관하게 항상 성립한다. 축 높이(+13mm)는 여기서 반지름을 빼는 방식으로 구한 값이라, 반지름이 바뀌면 축 높이만 따라 바뀌고 지면 위치(−20mm)는 그대로다.

### 바퀴 관성 텐서 (실린더 근사)

| 성분 | 값 (kg·m²) |
|---|---|
| 회전축(spin axis, 축 방향) | 1.91e-5 |
| 수직축 2개(축과 수직인 두 방향) | 1.15e-5 |

---

## 센서 장착 위치 (base_link 기준)

### LiDAR (RPLiDAR C1)

| 항목 | 값 |
|---|---|
| 장착 위치 (x, y, z) | (0, 0, 150mm) — 플랫폼 바닥면 기준 |
| 회전 방향/오프셋 | (0, 0, 0), 정면 기준 |
| 스캔 범위 | 0.05 ~ 12m, 360° |

### IMU (BNO055)

| 항목 | 값 |
|---|---|
| 장착 위치 (x, y, z) | (0, 0, 65mm) — 플랫폼 바닥면 기준 |

---

## Xacro 속성값 참조

`UGV_description/urdf/scout2map.urdf.xacro`에 이미 아래 값이 `xacro:property`로 반영되어 있다. 치수 변경 시 이 표와 함께 xacro 파일도 갱신한다. (단위: m, kg, rad)

```xml
<!-- Ground clearance: base_link origin (chassis underside) above the floor.
     Everything wheel-related is derived from this, not hard-coded. -->
<xacro:property name="ground_clearance" value="0.020"/>

<!-- Chassis -->
<xacro:property name="chassis_length" value="0.265"/>
<xacro:property name="chassis_width" value="0.220"/>
<xacro:property name="chassis_height" value="0.150"/>
<xacro:property name="chassis_mass" value="1.8"/>

<!-- Wheel -->
<xacro:property name="wheel_radius" value="0.033"/>
<xacro:property name="wheel_width" value="0.026"/>
<xacro:property name="wheel_mass" value="0.035"/>
<xacro:property name="track_width" value="0.240"/>
<xacro:property name="wheel_base" value="0.140"/>

<!-- Derived, not a literal: axle sits wheel_radius above the floor line,
     and the floor line is ground_clearance below base_link. -->
<xacro:property name="wheel_axle_z" value="${wheel_radius - ground_clearance}"/>

<!-- Sensor offsets (from base_link origin, platform bottom face) -->
<xacro:property name="lidar_offset_z" value="0.150"/>
<xacro:property name="imu_offset_z" value="0.065"/>
```

> `lidar_offset_z`(150mm)는 값 자체를 이번에 안 건드렸지만, `chassis_height`가 160mm→150mm로 줄면서 결과적으로 새 차체 상단면과 정확히 같은 높이가 됐다(예전엔 상단면보다 10mm 낮은 위치였음). LiDAR가 원래 상단 플레이트 위에 별도 마운트로 얹히는 구조라면 문제없지만, 실측 마운트 높이와 맞는지 한 번 확인해보는 게 좋다.

---

## 시뮬레이션 참고 사항

- 정밀 3D mesh 없이 box/cylinder primitive geometry로 진행. Gazebo 물리엔진은 `<collision>` geometry 기준으로 동작하므로 시각적 mesh는 추후 CAD 확정 시 교체해도 무방하다.
- 요철·슬립 지형은 Gazebo heightmap geometry(grayscale 이미지 기반 높낮이) 및 `<surface><friction><ode><mu>` 파라미터 조합으로 재현 가능함을 확인, `worlds/slip_test.world.sdf`에 반영했다. 요철 구간은 heightmap, 슬립 구간은 별도 저마찰(mu=0.05) 박스로 분리 구성되어 있다.
- 4륜 스키드 스티어는 `gz-sim-diff-drive-system` 플러그인에 `left_joint`/`right_joint`를 각각 2개(front+rear)씩 등록하는 방식으로 구현했다.
