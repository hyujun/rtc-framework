# rtc_inference

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

RTC 프레임워크의 **실시간 안전(RT-safe) ONNX Runtime 추론 엔진** 패키지입니다. 신경망 모델을 실시간 제어 루프에서 결정론적으로 실행할 수 있도록 설계된 헤더 전용(header-only) INTERFACE 라이브러리이며, 초기화 이후 동적 메모리 할당 없이 추론을 수행합니다.

---

## 핵심 컴포넌트

### ModelConfig (`inference_types.hpp`)

모델 설정을 담는 구조체입니다. `rtc` 네임스페이스에 정의되어 있습니다.

| 필드 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `model_path` | `std::string` | -- | ONNX 모델 파일 경로 |
| `optimized_model_path` | `std::string` | `""` | ORT 그래프 최적화 캐시 경로 (빈 문자열이면 비활성) |
| `input_shape` | `std::vector<int64_t>` | -- | 단일 입력 텐서 형상 (예: `{1, 12, 16}`) |
| `output_shapes` | `std::vector<std::vector<int64_t>>` | -- | 출력 head 별 형상. 단일 출력은 `{{1, 1, 13}}`, 3-head 모델은 `{{1, 1}, {1, 3}, {1, 3}}` |
| `intra_op_threads` | `int` | `1` | 추론 내부 스레드 수 (RT 환경에서는 단일 스레드 권장) |

엔진은 **단일 입력 / N 출력 head** 를 지원합니다 (multi-input 은 미지원 — 실제 소비자 수요에 맞춰 일반화).

---

### InferenceEngine (`inference_engine.hpp`)

모든 추론 백엔드의 추상 기반 클래스입니다. Non-copyable, non-movable이며 `rtc` 네임스페이스에 정의되어 있습니다.

| 메서드 | 반환 타입 | RT-safe | 설명 |
|--------|-----------|---------|------|
| `Init(const ModelConfig&)` | `void` | No | 모델 로드, 텐서 할당, 워밍업 (순수 가상) |
| `Run()` | `bool` | Yes | 모든 등록된 모델에 대해 추론 실행 (순수 가상) |
| `RunModel(int model_idx)` | `bool` | Yes | 단일 모델 추론 (기본 구현: `Run()` 위임) |
| `RunModels(const int*, int)` | `bool` | Yes | 복수 모델 배치 추론 (기본 구현: `RunModel()` 순차 호출) |
| `input_buffer(int model_idx)` | `float*` | Yes | 사전 할당된 입력 버퍼 포인터 반환 (범위 밖 → `nullptr`) |
| `output_buffer(int model_idx, int output_idx)` | `const float*` | Yes | 출력 head 버퍼 포인터 반환 (범위 밖 → `nullptr`) |
| `input_size(int model_idx)` | `std::size_t` | Yes | 입력 버퍼의 float 원소 수 (범위 밖 → `0`) |
| `output_size(int model_idx, int output_idx)` | `std::size_t` | Yes | 출력 head 버퍼의 float 원소 수 (범위 밖 → `0`) |
| `num_outputs(int model_idx)` | `int` | Yes | 모델의 출력 head 수 (범위 밖 → `0`) |
| `is_initialized()` | `bool` | Yes | 초기화 완료 여부 (순수 가상) |
| `num_models()` | `int` | Yes | 등록된 모델 수 (순수 가상) |

모든 RT-safe 메서드에는 `noexcept`가 지정되어 있으며, `Run()`, `RunModel()`, `RunModels()`에는 `[[nodiscard]]` 속성이 부여되어 반환값 무시를 방지합니다.

---

### OnnxEngine (`onnx/onnx_engine.hpp`)

`InferenceEngine`의 ONNX Runtime 구현체입니다. 컴파일 시 `HAS_ONNXRUNTIME` 매크로 정의 여부에 따라 전체 구현 또는 스텁이 선택됩니다.

#### 전체 구현 (`HAS_ONNXRUNTIME` 정의 시)

`Init()`을 여러 번 호출하여 복수의 모델을 순차적으로 등록할 수 있습니다. 각 모델은 `std::unique_ptr<Model>`로 보유되어 주소가 안정적이며 (텐서/IoBinding 의 버퍼·세션 참조가 `models_` 성장에도 무효화되지 않음), 첫 호출 시 `Ort::Env`를 생성하고 호출마다 다음 리소스를 사전 할당합니다:

- `Ort::Session` (모델당 1개)
- `Ort::IoBinding` (모델당 1개)
- `Ort::Value` 입력 텐서 1개 + 출력 head 별 텐서 N개
- `std::vector<float>` 입력 버퍼 1개 + 출력 head 별 버퍼 N개
- `Ort::RunOptions` (전체에서 1개, 재사용)

**RT 세션 옵션** (모델당): `ORT_SEQUENTIAL` 실행 모드, `intra_op_threads` 단일 스레드, `session.intra_op.allow_spinning=0` (intra-op 워커의 busy-spin 제거로 RT 루프 지터 차단), `ORT_ENABLE_ALL` 그래프 최적화.

**모델 검증**: `Init()`은 버퍼를 할당하기 전에 모델의 실제 입출력 arity/shape 를 `config` 와 대조합니다. 출력 head 수가 `config.output_shapes`와 정확히 일치하지 않거나(부족·과다 모두 거부 — positional binding 이라 extra head 는 조용히 drop 되므로) 정적 차원이 불일치하면 `std::runtime_error`를 던져 잘못된/재학습된 `.onnx`가 런타임에 조용히 틀린 값을 내지 않고 setup 단계에서 큰 소리로 실패합니다 (모델 쪽 dim `< 0`인 dynamic dim 은 임의 크기와 매칭). `config`에 선언된 shape 의 각 dim 도 static positive 여야 하며 `<= 0`이면 즉시 reject — 그렇지 않으면 dynamic `-1`이 버퍼 크기 계산(`Numel()`)에서 `SIZE_MAX`로 캐스트되어 catastrophic allocation을 시도할 수 있습니다. 워밍업 추론은 모델을 `models_`에 등록하기 **전에** 실행됩니다 (`RunModels()`와 동일한 direct `Session::Run` 경로 재사용 → warmup 이 production 경로를 정확히 데움) — 이 순서 덕분에 `Init()`이 원자적입니다: 워밍업이 예외를 던지면 반쯤 초기화된 모델이 `models_`에 남지 않고 그대로 전파됩니다 (register-or-nothing).

**`Reset()`** (non-RT): 등록된 모든 모델을 해제하여 `Init()` 재호출을 idempotent 하게 만듭니다 (`Ort::Env`/`RunOptions`는 재사용). 같은 엔진 인스턴스로 재초기화하는 소비자는 `Init()` 루프 전에 `Reset()`을 호출해 모델 중복 등록·세션 누수를 방지합니다.

**추론 실행 경로:**

| 메서드 | 동작 방식 | 비고 |
|--------|-----------|------|
| `Run()` | IoBinding 기반 순차 실행 (`SynchronizeInputs/Outputs` 호출) | 모든 모델 실행 |
| `RunModel(idx)` | IoBinding 기반 단일 모델 실행 | 후방 호환용 |
| `RunModels(indices, count)` | `Session::Run()` 직접 호출 (IoBinding 우회) | RT 루프 권장, 가장 빠름 |

`RunModels()`가 가장 빠른 이유는 IoBinding 래퍼의 `SynchronizeInputs/Outputs` 오버헤드를 우회하기 때문입니다. CPU 백엔드에서 이 동기화는 no-op이므로 생략해도 안전합니다.

**RT 안전 메커니즘:**
- 모든 RT 메서드 내부에서 `try-catch`로 ONNX Runtime 예외를 포착하여 `false`를 반환
- `RunModel()`, `RunModels()`는 인덱스 범위를 검증하고 범위 밖이면 `false` 반환
- 모든 접근자 (`input_buffer`/`output_buffer`/`input_size`/`output_size`/`num_outputs`) 도 범위를 검증하여 OOR 시 `nullptr`/`0` 반환 (RT-safe, 예외 없음)
- `Run()`/`RunModel()`/`RunModels()` 모두 사전 할당된 단일 `Ort::RunOptions`를 재사용
- Non-copyable, non-movable (단일 스레드 소유 전제, 스레드 안전 보장 없음)

#### 스텁 구현 (`HAS_ONNXRUNTIME` 미정의 시)

ONNX Runtime이 설치되지 않은 환경에서도 빌드가 가능하도록 모든 메서드가 no-op 또는 더미 값을 반환합니다.

| 메서드 | 스텁 동작 |
|--------|-----------|
| `Init()` | no-op |
| `Run()` | `false` 반환 |
| `input_buffer()` / `output_buffer()` | `nullptr` 반환 |
| `input_size()` / `output_size()` | `0` 반환 |
| `is_initialized()` | `false` 반환 |
| `num_models()` | `0` 반환 |

호출 코드에서 `is_initialized()`를 확인하면 스텁 모드에서 안전하게 추론을 건너뛸 수 있습니다.

---

## 빌드 동작

이 패키지는 INTERFACE 라이브러리이므로 자체 바이너리를 생성하지 않습니다. 소비자 패키지에서 `find_package(rtc_inference)`로 의존하면 헤더 경로와 ONNX Runtime 링크가 자동으로 전파됩니다.

### ONNX Runtime 감지 (CMakeLists.txt)

CMake에서 2단계로 ONNX Runtime을 탐색합니다:

1. **CMake 패키지 탐색:** `find_package(onnxruntime QUIET)`
2. **수동 탐색 (폴백):**
   - `/opt/onnxruntime/onnxruntime-*` (버전별 하위 디렉토리 자동 탐색)
   - `/opt/onnxruntime`, `/usr/local`, `/usr/lib/x86_64-linux-gnu`, `/usr/lib/aarch64-linux-gnu`
   - 헤더: `include/onnxruntime/core/session`, `include/onnxruntime` 접미사로 `onnxruntime_cxx_api.h` 탐색

| 감지 결과 | 동작 |
|-----------|------|
| 발견 | `HAS_ONNXRUNTIME` 컴파일 정의 전파 + 라이브러리 링크 (INTERFACE) |
| 미발견 | 스텁 엔진으로 빌드 (빌드 실패 없음) |

### 빌드 명령

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select rtc_inference
```

---

## 의존성

| 의존성 | 종류 | 용도 |
|--------|------|------|
| `ament_cmake` | 빌드 도구 | ROS 2 빌드 시스템 |
| `rtc_base` | 런타임 의존 | 공유 데이터 타입 |
| `libonnxruntime-dev` | 빌드 의존 (선택) | ONNX Runtime 백엔드 |
| (lint depend 없음) | — | 워크스페이스 정책 (`bdedac7`): `ament_lint_common` meta / `ament_uncrustify` 사용 금지 — 필요 시 개별 `ament_cmake_{cppcheck,lint_cmake,xmllint}` 만 추가. 자세한 사유: [agent_docs/conventions.md](../agent_docs/conventions.md) |

C++ 20 표준이 요구됩니다 (`CMAKE_CXX_STANDARD 20`).

---

## 테스트

```bash
colcon test --packages-select rtc_inference --event-handlers console_direct+
colcon test-result --verbose
```

`test/test_inference_engine.cpp` (`test_inference_engine`, `BUILD_TESTING`)가 `InferenceEngine`의 기본 델리게이트 동작(`RunModel`/`RunModels`)과 미초기화·범위 밖 접근자의 안전한 fallback을 검증하며, `HAS_ONNXRUNTIME` 빌드에서는 실제 `OnnxEngine` 경로도 함께 검증됩니다.

---

## 라이선스

MIT License -- [LICENSE](../LICENSE) 참조.
