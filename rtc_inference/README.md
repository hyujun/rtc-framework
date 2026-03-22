# rtc_inference

![version](https://img.shields.io/badge/version-v5.16.0-blue)

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **실시간 안전(RT-safe) 추론 엔진 추상화** 패키지입니다. 신경망 모델을 500 Hz 실시간 제어 루프에서 결정론적으로 실행할 수 있도록 설계되었으며, 현재 ONNX Runtime 백엔드를 지원합니다.

**핵심 설계 원칙:**
- 헤더 전용(header-only) + INTERFACE 라이브러리
- 초기화 후 동적 할당 없음 — RT-safe 추론 경로
- 모든 RT-safe 메서드에 `noexcept` 보장
- ONNX Runtime 미설치 시 스텁(stub) 자동 제공 — 빌드 실패 없음
- 다중 모델 배치 추론 지원

---

## 패키지 구조

```
rtc_inference/
├── CMakeLists.txt
├── package.xml
└── include/rtc_inference/
    ├── inference_types.hpp        ← 모델 설정 구조체 (ModelConfig)
    ├── inference_engine.hpp       ← 추론 엔진 추상 인터페이스
    └── onnx/
        └── onnx_engine.hpp        ← ONNX Runtime 구현체 + 스텁
```

---

## 아키텍처

```
┌──────────────────────────────────────────────────────────┐
│  ur5e_hand_driver                                        │
│  └─ FingertipFTInferencer : OnnxEngine                   │
│     - InitFT() → 핑거팁별 모델 등록                       │
│     - Infer() → 전처리 → 배치 추론 → 결과 복사            │
└──────────────┬───────────────────────────────────────────┘
               │ 상속
┌──────────────▼───────────────────────────────────────────┐
│  OnnxEngine : InferenceEngine                            │
│  - Init() → 세션 생성 + 텐서 사전 할당 + IoBinding         │
│  - RunModels() → 배치 추론 (할당 없음)                    │
└──────────────┬───────────────────────────────────────────┘
               │ 상속
┌──────────────▼───────────────────────────────────────────┐
│  InferenceEngine (추상 기반 클래스)                       │
│  - Init(), Run(), RunModel(), RunModels()                │
│  - input_buffer(), output_buffer()                       │
└──────────────────────────────────────────────────────────┘
```

---

## 컴포넌트 상세

### ModelConfig (`inference_types.hpp`)

모델 설정 구조체입니다.

| 필드 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `model_path` | `string` | — | ONNX 모델 파일 경로 |
| `optimized_model_path` | `string` | `""` | 그래프 최적화 캐시 경로 (빈 값 = 비활성) |
| `input_shape` | `vector<int64_t>` | — | 입력 텐서 형상 (예: `{1, 12, 16}`) |
| `output_shape` | `vector<int64_t>` | — | 출력 텐서 형상 (예: `{1, 1, 13}`) |
| `intra_op_threads` | `int` | `1` | 추론 스레드 수 (RT: 단일 스레드 권장) |

---

### InferenceEngine (`inference_engine.hpp`)

모든 추론 백엔드의 추상 기반 클래스입니다. Non-copyable, non-movable.

#### 추상 메서드

| 메서드 | 반환 | RT-safe | 설명 |
|--------|------|---------|------|
| `Init(const ModelConfig&)` | `void` | No | 모델 로드 + 텐서 할당 + 워밍업 |
| `Run()` | `bool` | **Yes** | 모든 모델 추론 실행 |
| `RunModel(int idx)` | `bool` | **Yes** | 단일 모델 추론 |
| `RunModels(const int*, int)` | `bool` | **Yes** | 배치 모델 추론 |
| `input_buffer(int idx)` | `float*` | **Yes** | 입력 버퍼 접근 |
| `output_buffer(int idx)` | `const float*` | **Yes** | 출력 버퍼 접근 |
| `input_size(int idx)` | `size_t` | **Yes** | 입력 버퍼 크기 |
| `output_size(int idx)` | `size_t` | **Yes** | 출력 버퍼 크기 |
| `is_initialized()` | `bool` | **Yes** | 초기화 상태 |
| `num_models()` | `int` | **Yes** | 로드된 모델 수 |

---

### OnnxEngine (`onnx/onnx_engine.hpp`)

`InferenceEngine`의 ONNX Runtime 구현체입니다. 컴파일 시 `HAS_ONNXRUNTIME` 플래그에 따라 전체 구현 또는 스텁이 선택됩니다.

#### 전체 구현 (`HAS_ONNXRUNTIME` 정의 시)

**사전 할당 전략 — 초기화 시 모든 리소스 할당:**

| 리소스 | 설명 |
|--------|------|
| `Ort::Env` | ONNX Runtime 환경 (첫 Init 시 1회 생성) |
| `Ort::RunOptions` | 추론 옵션 (재사용) |
| `Ort::Session` | 모델당 1개 세션 |
| `Ort::IoBinding` | 모델당 1개 I/O 바인딩 |
| `Ort::Value` (input/output) | 사전 할당 텐서 |
| `vector<float>` (input/output) | 사전 할당 데이터 버퍼 |

**추론 실행 경로:**

| 경로 | 메서드 | 최적화 | 권장 용도 |
|------|--------|--------|----------|
| 단일 모델 | `RunModel(idx)` | IoBinding 기반 (SynchronizeInputs/Outputs 호출) | 후방 호환 |
| 배치 모델 | `RunModels(indices, count)` | **직접 `Session::Run()`** (IoBinding 우회, 동기화 생략) | **RT 루프 권장** |
| 전체 모델 | `Run()` | IoBinding 기반 순차 호출 | 모든 모델 실행 |

> **`RunModels()`가 가장 빠른 이유:** IoBinding 래퍼와 SynchronizeInputs/Outputs 오버헤드를 우회하여 ONNX Runtime `Session::Run()`을 직접 호출합니다. CPU 백엔드에서 동기화는 no-op이므로 안전하게 생략됩니다.

**RT 안전 메커니즘:**
- 모든 RT 메서드는 내부에서 `try-catch`로 감싸져 ONNX Runtime 예외를 `false` 반환으로 변환합니다
- `RunModel()`, `RunModels()`는 인덱스 범위 검증 후 `false` 반환
- `output_buffer()`는 `const float*` 반환 (읽기 전용)
- `OnnxEngine`은 Non-copyable, Non-movable (소유권 단일 스레드)
- 스레드 안전 보장 없음 — 단일 스레드 소비자 전제

```cpp
// 사용 예시
OnnxEngine engine;

// 비-RT 초기화
ModelConfig config{
    .model_path = "/path/to/model.onnx",
    .input_shape = {1, 12, 16},
    .output_shape = {1, 1, 13},
    .intra_op_threads = 1
};
engine.Init(config);  // 모델 로드 + 텐서 할당

// 여러 모델 등록 (순차 Init 호출)
engine.Init(config2);
engine.Init(config3);

// RT 루프 (500 Hz)
float* in = engine.input_buffer(0);
std::copy_n(sensor_data, engine.input_size(0), in);  // 입력 채우기

int indices[] = {0, 1, 2};
engine.RunModels(indices, 3);  // 배치 추론 (할당 없음)

const float* out = engine.output_buffer(0);  // 결과 읽기
```

#### 스텁 구현 (`HAS_ONNXRUNTIME` 미정의 시)

ONNX Runtime이 설치되지 않은 환경에서도 빌드가 가능하도록 모든 메서드가 no-op 또는 더미 값을 반환합니다.

| 메서드 | 동작 |
|--------|------|
| `Init()` | no-op (조용히 성공) |
| `Run()`, `RunModel()`, `RunModels()` | `false` 반환 |
| `input_buffer()`, `output_buffer()` | `nullptr` 반환 |
| `is_initialized()` | `false` 반환 |
| `num_models()` | `0` 반환 |

> 호출 코드에서 `is_initialized()`를 확인하면 스텁 모드에서 안전하게 추론을 건너뛸 수 있습니다.

---

## ONNX Runtime 감지 로직

CMakeLists.txt에서 2단계 감지를 수행합니다:

1. **CMake 패키지 탐색:** `find_package(onnxruntime QUIET)`
2. **수동 탐색 (폴백):**
   - 라이브러리: `/opt/onnxruntime/lib`, `/usr/local/lib`, `/usr/lib/x86_64-linux-gnu`, `/usr/lib/aarch64-linux-gnu`
   - 헤더: `/opt/onnxruntime/include`, `/usr/include/onnxruntime`, `/usr/local/include/onnxruntime`

감지 결과:
- **발견:** `HAS_ONNXRUNTIME` 컴파일 정의 + 라이브러리 링크 (INTERFACE로 소비자에게 전파)
- **미발견:** `ONNX Runtime not found — building stub inference engine` 메시지 출력

---

## 핑거팁 F/T 추론 (ur5e_hand_driver 연동)

`ur5e_hand_driver`의 `FingertipFTInferencer`가 `OnnxEngine`을 상속하여 핑거팁 촉각/힘 추론을 수행합니다.

### 데이터 흐름

```
핸드 센서 데이터 (int32_t[88])
    ↓
Bessel 저역통과 필터
    ↓
FingertipFTInferencer::Infer()  [RT-safe, 500 Hz]
    ├─ Phase 1: 정규화 + 델타 계산 + FIFO 히스토리 시프트
    ├─ Phase 2: RunModels() — 배치 ONNX 추론 (할당 없음)
    └─ Phase 3: 결과 → FingertipFTState 복사
    ↓
SeqLock으로 스레드 안전 퍼블리시
    ↓
소비자: GetLatestFTState() → FingertipFTState
```

### 입출력 형상

| 항목 | 형상 | 설명 |
|------|------|------|
| 입력 | `{1, 12, 16}` | 12행(히스토리) × 16채널(barometer[8] + delta[8]) |
| 출력 | `{1, 1, 13}` | 13개 값: contact(1) + F(3) + u(3) + Fn(3) + Fx(1) + Fy(1) + Fz(1) |

### 캘리브레이션

`FeedCalibration()`으로 기준선 오프셋을 자동 측정합니다 (기본 500 샘플 @500 Hz = 1초).

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rtc_base` | 공유 데이터 타입 |
| `libonnxruntime-dev` | ONNX Runtime (선택적 빌드 의존성) |

---

## 빌드

```bash
# ONNX Runtime 설치 (선택)
# Ubuntu: sudo apt install libonnxruntime-dev
# 또는: /opt/onnxruntime에 수동 설치

cd ~/ur_ws
colcon build --packages-select rtc_inference
source install/setup.bash
```

INTERFACE 라이브러리이므로 자체 바이너리는 생성되지 않습니다. 소비자 패키지에서 `#include`하면 ONNX Runtime 연결이 자동 전파됩니다.

---

## 의존성 그래프 내 위치

```
rtc_base
    ↓
rtc_inference  ← 추론 엔진 추상화 (ONNX 백엔드)
    ↑
    └── ur5e_hand_driver  (FingertipFTInferencer로 F/T 추론)
```

---

## 향후 확장

`InferenceEngine` 추상화를 통해 추가 백엔드를 지원할 수 있습니다:

| 백엔드 | 상태 | 용도 |
|--------|------|------|
| ONNX Runtime | **구현 완료** | CPU 기반 범용 추론 |
| TensorRT | 계획 | NVIDIA GPU 가속 |
| OpenVINO | 계획 | Intel CPU/GPU 최적화 |

---

## 최적화 내역 (v5.16.1)

| 영역 | 변경 내용 |
|------|----------|
| **CMakeLists.txt** | C++20 표준 명시 — INTERFACE 라이브러리 소비자에게 요구사항 전파 |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
