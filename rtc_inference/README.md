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
| `inputs` | `std::vector<TensorSpec>` | -- | 입력 텐서. 순서 = 소비자가 `input_buffer(m, i)` 로 인덱싱할 순서 |
| `outputs` | `std::vector<TensorSpec>` | -- | 출력 head. 순서 = `output_buffer(m, o)` 인덱스 |
| `intra_op_threads` | `int` | `1` | 추론 내부 스레드 수 (RT 환경에서는 단일 스레드 권장) |

`TensorSpec` 은 `{ std::string name; std::vector<int64_t> shape; }` 입니다. 엔진은 **N 입력 × N 출력 head** 를 지원합니다 (#511 P2).

#### 텐서를 이름으로 바인딩하는 이유

**positional 바인딩은 같은 shape 인 두 텐서를 구분하지 못합니다.** 가정이 아니라 실제 사례가 있습니다 — LSTM 정책의 `h_out`/`c_out` 은 정의상 동일 shape 이고, `udp_hand_driver` 의 fingertip F/T 모델은 `{{1,1},{1,3},{1,3}}` 로 힘과 방향이 둘 다 `[1,3]` 입니다. 그런 모델이 두 텐서를 반대 순서로 export 되면 **모든 shape 검사를 통과한 채 값이 조용히 뒤바뀝니다.**

그래서 각 side (입력 / 출력) 는 다음 셋 중 하나입니다:

| 선언 | 바인딩 | 비고 |
|---|---|---|
| 모든 텐서에 이름 | **이름 기준** | 위 뒤바뀜을 검출한다. `config` 순서가 모델 순서와 달라도 되며, 슬롯 i 는 자기 이름의 텐서를 받는다 |
| 모든 텐서 이름이 빈 문자열 | **positional** | 모델의 텐서 이름을 모르는 소비자를 위한 호환 경로. 위 사각을 그대로 물려받는다 |
| 일부만 이름 | **거부** | 의도한 바인딩이 두 가지로 갈리는 유일한 경우라, 추측하지 않고 `Init()` 이 실패한다 |

---

### InferenceEngine (`inference_engine.hpp`)

모든 추론 백엔드의 추상 기반 클래스입니다. Non-copyable, non-movable이며 `rtc` 네임스페이스에 정의되어 있습니다.

| 메서드 | 반환 타입 | RT-safe | 설명 |
|--------|-----------|---------|------|
| `Init(const ModelConfig&)` | `void` | No | 모델 로드, 텐서 할당, 워밍업 (순수 가상) |
| `Run()` | `bool` | Yes | 모든 등록된 모델에 대해 추론 실행 (순수 가상) |
| `RunModel(int model_idx)` | `bool` | Yes | 단일 모델 추론 (기본 구현: `Run()` 위임) |
| `RunModels(const int*, int)` | `bool` | Yes | 복수 모델 배치 추론 (기본 구현: `RunModel()` 순차 호출) |
| `input_buffer(int model_idx, int input_idx)` | `float*` | Yes | 사전 할당된 입력 버퍼 포인터 (범위 밖 → `nullptr`) |
| `output_buffer(int model_idx, int output_idx)` | `const float*` | Yes | 출력 head 버퍼 포인터 반환 (범위 밖 → `nullptr`) |
| `input_size(int model_idx, int input_idx)` | `std::size_t` | Yes | 입력 버퍼의 float 원소 수 (범위 밖 → `0`) |
| `output_size(int model_idx, int output_idx)` | `std::size_t` | Yes | 출력 head 버퍼의 float 원소 수 (범위 밖 → `0`) |
| `num_inputs(int model_idx)` | `int` | Yes | 모델의 입력 텐서 수 (범위 밖 → `0`) |
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
- `Ort::Value` 입력 텐서 N개 + 출력 head 별 텐서 N개
- `std::vector<float>` 입력 버퍼 N개 + 출력 head 별 버퍼 N개 (전부 **선언 순서**로 인덱싱)
- `Ort::RunOptions` (전체에서 1개, 재사용)

**RT 세션 옵션** (모델당): `ORT_SEQUENTIAL` 실행 모드, `intra_op_threads` 단일 스레드, `session.intra_op.allow_spinning=0` (intra-op 워커의 busy-spin 제거로 RT 루프 지터 차단), `ORT_ENABLE_ALL` 그래프 최적화.

**모델 검증**: `Init()`은 버퍼를 할당하기 전에 모델의 실제 입출력 arity/shape 를 `config` 와 대조하고, 불일치가 하나라도 있으면 **전량을 표로 담은** `std::runtime_error`를 던집니다 — 잘못된/재학습된 `.onnx`가 런타임에 조용히 틀린 값을 내지 않고 setup 단계에서 큰 소리로 실패합니다.

비교 규칙은 [shape_report.hpp](include/rtc_inference/shape_report.hpp) 가 SSoT 이며, 요지는 셋입니다 — ① **입력·출력 양쪽 arity 가 정확히 일치**해야 한다 (한쪽에만 있는 텐서는 각각 "config 미선언" / "모델에 없음" 행으로 실패; extra head 는 조용히 drop 되고, 미선언 입력은 ORT 내부 에러로 죽어 원인을 안 가리킨다), ② 모델 쪽 dim `< 0` 은 dynamic 이라 임의 크기와 매칭되지만 **`config` 쪽 dim 은 static positive** 여야 한다 (dynamic `-1`이 `Numel()`에서 `SIZE_MAX`로 캐스트되어 catastrophic allocation 을 시도하므로 — 이 검사가 rank 검사보다 먼저다), ③ **한 번에 전부 보고**한다 (재학습은 여러 텐서를 동시에 옮기므로 첫 불일치에서 throw 하면 텐서마다 bring-up 사이클을 하나씩 태운다).

> **positional 로 선언한 경우의 사각**: 모든 이름을 빈 문자열로 둔 side 는 positional 로 바인딩되므로 **같은 shape 두 텐서가 뒤바뀐 export 를 검출하지 못합니다.** `udp_hand_driver` 의 3-head FT 모델(`{{1,1},{1,3},{1,3}}` — 힘과 방향이 둘 다 `[1,3]`)이 현재 이 경로에 있습니다 (실모델이 in-tree 에 없어 이름을 채우지 못했다 — 채우는 순간 엔진이 검증한다). 표는 어느 side 가 어떤 방식으로 짝지어졌는지 (`paired by name` / `paired positionally`) 를 함께 찍으므로, 깨끗해 보이는 행이 실제로 깨끗한지 판별할 수 있습니다.

검증은 **두 겹**으로 테스트됩니다. `test_shape_report.cpp` 는 ORT 비의존 순수 함수를 직접 몰아 비교 **규칙**을 전량 고정하고, `test_onnx_engine.cpp` 는 커밋된 `.onnx` 픽스처를 실제 세션에 올려 그 규칙이 실제 버퍼에 **배선**됐는지 확인합니다. 둘은 서로를 대체하지 않습니다 — 완벽한 비교기가 엉뚱한 버퍼에 물려 있으면 여전히 조용히 틀린 로봇입니다.

픽스처(`test/data/two_in_two_out.onnx`, 173 B)는 **생성하지 않고 커밋**합니다: 생성에 python `onnx` 패키지가 필요한데 이 저장소의 의존이 아니므로, 테스트 시점에 만들면 그 패키지가 있는 곳에서만 green 이고 없는 곳에서는 조용히 건너뛰는 host-gated 게이트가 됩니다. 재생성 절차는 [test/data/generate_fixtures.py](test/data/generate_fixtures.py) 헤더에 있습니다. 픽스처의 네 텐서가 **전부 `[1,2]`** 인 것이 요점입니다 — shape 으로는 구분이 불가능하므로, 그 테스트가 올바른 값을 얻는 경로는 이름 바인딩뿐입니다.

워밍업 추론은 모델을 `models_`에 등록하기 **전에** 실행됩니다 (`RunModels()`와 동일한 direct `Session::Run` 경로 재사용 → warmup 이 production 경로를 정확히 데움) — 이 순서 덕분에 `Init()`이 원자적입니다: 워밍업이 예외를 던지면 반쯤 초기화된 모델이 `models_`에 남지 않고 그대로 전파됩니다 (register-or-nothing).

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
- 모든 접근자 (`input_buffer`/`output_buffer`/`input_size`/`output_size`/`num_inputs`/`num_outputs`) 도 범위를 검증하여 OOR 시 `nullptr`/`0` 반환 (RT-safe, 예외 없음)
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

## 오프라인 모델 검사 (`rtc_inference_check`)

`.onnx` 가 실제로 무엇을 노출하는지 bring-up 없이 확인합니다. ONNX Runtime 이 있을 때만 빌드됩니다.

```bash
# 모델이 무엇을 노출하는지 덤프
ros2 run rtc_inference rtc_inference_check policy.onnx

# 선언과 대조 (불일치 시 표를 찍고 exit 1)
ros2 run rtc_inference rtc_inference_check policy.onnx \
    --input obs:1x34 --output action:1x6

# 한쪽만 선언해도 된다 — 선언한 쪽만 판정하고 나머지는 덤프한다
ros2 run rtc_inference rtc_inference_check policy.onnx --input obs:1x34
```

이름 없는 텐서(positional 바인딩)는 `:1x34` 처럼 이름을 비워 선언합니다.

`--input` / `--output` 은 독립적이므로 config 를 점진적으로 쓰면서 한쪽씩 확인할 수 있습니다. 선언하지 않은 쪽은 **판정에 들어가지 않고** 덤프만 됩니다 — 빈 선언을 그대로 대조에 넣으면 "config 가 이 텐서들을 하나도 선언하지 않았다" 로 읽혀 일치하는 모델에 불일치가 찍힙니다.

**이것은 두 번째 검증기가 아닙니다.** 권위 있는 검사는 여전히 configure 시점의 `OnnxEngine::Init` 이고, 이 도구는 **같은 `CompareModelIo` 를 호출해 같은 `IoReport::Format` 으로 렌더**합니다 — 규칙을 재구현하지 않으므로 둘이 갈라질 수 없습니다 (AP-DOC-1). 컨트롤러 YAML 을 읽지 않는 것도 같은 이유입니다: 그 스키마의 파서는 `rtc_controllers` 가 소유하며, 여기에 두 번째 YAML 파서를 두면 그 규칙이 복제됩니다.

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
