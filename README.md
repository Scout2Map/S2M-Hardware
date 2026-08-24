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

모든 치수는 `base_link` 원점을 기준으로 한다[cite: 3]. `base_link` 원점은 **지면(z=0)** 으로 정의하며, 차체 플랫폼 바닥면은 이 지면에서 20mm 띄워진다[cite: 3].

- x축: 전진 방향 (+)[cite: 3]
- y축: 좌측 방향 (+)[cite: 3]
- z축: 상방 (+)[cite: 3]

지면 기준을 먼저 고정한다: `base_link` 가 지면 자체이므로, 차체 바닥면은 지면에서 20mm 띄워진다 ( `ground_clearance` )[cite: 3]. 바퀴는 반드시 이 지면에 접지해야 하므로 바퀴 축 높이는 `wheel_radius` (현재 33mm) 와 동일하게 설정된다[cite: 3]. 타이어를 바꿔 `wheel_radius` 가 달라져도 바퀴는 항상 지면 ( `z=0` ) 에 닿도록 xacro에 수식으로 넣어뒀다[cite: 3].

---

## 차체 본체 (base_link)

| 항목 | 값 |
|---|---|
| 크기 (L × W × H) | 265mm × 220mm × 110mm |
| 지면 간격 (ground clearance, 바닥면 → 지면) | 20mm[cite: 3] |
| 지면 기준 전체 높이 (지면 → 차체 상단) | 130mm (20 + 110) |
| 질량 | 1800g[cite: 3] |
| 무게중심 오프셋 (x, y, z) | (0, 0, 75mm) — 기하학적 중심 근사치, 배터리/SBC 배치 확정 시 보정 필요[cite: 3] |
| 관성 텐서 | 아래 표 참조 (박스 근사)[cite: 3] |

### 관성 텐서 (box 근사, 무게중심 기준)

| 성분 | 값 (kg·m²) |
|---|---|
| I_xx | 0.009075 |
| I_yy | 0.012349 |
| I_zz | 0.017794 |
| I_xy, I_xz, I_yz | 0 (좌우/전후 대칭 가정)[cite: 3] |

> H가 150mm→110mm로 바뀌면서 전체 관성 텐서 값을 110mm 기준으로 재계산했다[cite: 3].

---

## 바퀴 (wheel_link)

| 항목 | 값 |
|---|---|
| 바퀴 지름 | 66mm (반지름 33mm)[cite: 3] |
| 바퀴 폭 | 26mm[cite: 3] |
| 바퀴 질량 | 개당 35g[cite: 3] |
| 트랙폭 (좌우 바퀴 중심 간 거리) | 240mm[cite: 3] |
| 휠베이스 (전후 바퀴 중심 간 거리) | 140mm[cite: 3] |
| 지면 간격 (ground clearance, 플랫폼 바닥면 → 지면) | 20mm[cite: 3] |
| 바퀴 축 높이 (지면 → 바퀴 축 중심, = 반지름) | +33mm |

### 바퀴 장착 위치 (지면 base_link 기준, 4륜 스키드 스티어)

| 바퀴 | x | y | z |
|---|---|---|---|
| front_left | +70mm[cite: 3] | +120mm[cite: 3] | +33mm |
| front_right | +70mm[cite: 3] | −120mm[cite: 3] | +33mm |
| rear_left | −70mm[cite: 3] | +120mm[cite: 3] | +33mm |
| rear_right | −70mm[cite: 3] | −120mm[cite: 3] | +33mm |

---

## 센서 장착 위치 (지면 base_link 기준)

### LiDAR (RPLiDAR C1)

| 항목 | 값 |
|---|---|
| 장착 위치 (x, y, z) | (0, 0, 130mm) — 지면 기준 |
| 회전 방향/오프셋 | (0, 0, 3.14159265), 정면 기준 180도 회전 보정 |
| 스캔 범위 | 0.05 ~ 12m, 360°[cite: 3] |

### IMU (BNO055)

| 항목 | 값 |
|---|---|
| 장착 위치 (x, y, z) | (0, 0, 85mm) — 지면 기준 |

---

## Xacro 속성값 참조

`UGV_description/urdf/scout2map.urdf.xacro` 에 이미 아래 값이 `xacro:property` 로 반영되어 있다[cite: 3]. 치수 변경 시 이 표와 함께 xacro 파일도 갱신한다[cite: 3]. (단위: m, kg, rad)[cite: 3]

```xml
<!-- ground clearance defines the gap below chassis -->
<xacro:property name="ground_clearance" value="0.020"/>

<!-- chassis dimensions -->
<xacro:property name="chassis_length" value="0.265"/>
<xacro:property name="chassis_width"  value="0.220"/>
<!-- chassis height is 110mm to reach 130mm from ground -->
<xacro:property name="chassis_height" value="0.110"/>
<xacro:property name="chassis_mass"   value="1.8"/>

<!-- base_link is now on the ground -->
<!-- wheel axle height is exactly the wheel radius -->
<xacro:property name="wheel_axle_z" value="${wheel_radius}"/>

<!-- sensor offsets relative to the ground -->
<!-- lidar sits on top of chassis at 130mm from ground -->
<xacro:property name="lidar_offset_z" value="0.130"/>
<!-- imu offset includes ground clearance -->
<xacro:property name="imu_offset_z"   value="0.085"/>

<!-- rotated using pi to fix orientation -->
<xacro:property name="lidar_yaw_offset" value="3.14159265"/>

---

## 시뮬레이션 참고 사항

- 정밀 3D mesh 없이 box/cylinder primitive geometry로 진행. Gazebo 물리엔진은 `<collision>` geometry 기준으로 동작하므로 시각적 mesh는 추후 CAD 확정 시 교체해도 무방하다.
- 요철·슬립 지형은 Gazebo heightmap geometry(grayscale 이미지 기반 높낮이) 및 `<surface><friction><ode><mu>` 파라미터 조합으로 재현 가능함을 확인, `worlds/slip_test.world.sdf`에 반영했다. 요철 구간은 heightmap, 슬립 구간은 별도 저마찰(mu=0.05) 박스로 분리 구성되어 있다.
- 4륜 스키드 스티어는 `gz-sim-diff-drive-system` 플러그인에 `left_joint`/`right_joint`를 각각 2개(front+rear)씩 등록하는 방식으로 구현했다.
