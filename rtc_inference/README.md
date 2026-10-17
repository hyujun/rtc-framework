# rtc_inference

ONNX Runtime 으로 신경망 모델을 실행하는 header-only 추론 엔진. **유형**: 라이브러리 패키지

## 개요

신경망 모델을 제어 루프에서 실행하기 위한 INTERFACE 라이브러리다. 세션·텐서 버퍼·워밍업은 `Init()` 에서 끝나고, 추론 호출은 `noexcept` 이며 래퍼 자신은 할당하지 않는다. **단 ONNX Runtime 의 `Run()` 은 내부에서 매 호출 heap 을 할당한다** — RT 경로에서 호출하려면 [invariants.md](../agent_docs/invariants.md) RT 절의 조건부 수용 조건 (RT-1 알려진 위반 2건째, 조건에 RT 프로세스의 `rtc::ConfigureRtHeap()` 포함) 을 만족해야 한다. 실측 도구는 `integrated_bringup` 의 `test_demo_inference_real_model` 이다.

소비자: [integrated_bringup](../integrated_bringup/README.md) 의 `demo_inference_controller` (`.onnx` 경로는 그 패키지의 컨트롤러 YAML 이 정한다). RT 최적화 배경: [rt-optimization.md](../docs/rt-optimization.md).

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. 이 패키지만 빌드·테스트하려면 `./build.sh --tests -p rtc_inference`.

소비자 `CMakeLists.txt` 에서 `find_package` 하면 헤더 경로와 ONNX Runtime 링크가 함께 전파된다.

```cmake
find_package(rtc_inference REQUIRED)
target_link_libraries(my_target PRIVATE rtc_inference)
```

```cpp
#include "rtc_inference/onnx/onnx_engine.hpp"

rtc::OnnxEngine engine;                       // non-RT: 설정 단계
engine.Init(rtc::ModelConfig{
    .model_path = "policy.onnx",
    .inputs = {{"obs", {1, 34}}},
    .outputs = {{"action", {1, 6}}}});        // 불일치면 std::runtime_error
// RT: 입력 채우기 → 추론 → 출력 읽기
float* in = engine.input_buffer(0, 0);
const int models[] = {0};
if (engine.RunModels(models, 1)) { const float* out = engine.output_buffer(0, 0); }
```

모델이 노출하는 텐서는 bring-up 없이 `rtc_inference_check` 로 확인한다 ([오프라인 모델 검사](#오프라인-모델-검사-rtc_inference_check)).

## 인터페이스

| 헤더 | 주요 타입 | 역할 |
|---|---|---|
| `rtc_inference/inference_types.hpp` | `ModelConfig` · `TensorSpec` | 모델 경로·입출력 텐서 선언 |
| `rtc_inference/inference_engine.hpp` | `InferenceEngine` | 백엔드 추상 기반 (non-copyable) |
| `rtc_inference/onnx/onnx_engine.hpp` | `OnnxEngine` | ONNX Runtime 구현. ORT 가 없으면 스텁 |
| `rtc_inference/shape_report.hpp` | `CompareModelIo` · `IoReport` | ORT 비의존 입출력 대조·표 렌더 |

필드·메서드 상세와 RT 여부는 각 헤더의 Doxygen 이 SSoT 다. 아래는 사용 계약만 요약한다.

- `ModelConfig`: `model_path`, `optimized_model_path` (빈 문자열이면 그래프 최적화 캐시 비활성), `inputs`/`outputs` (`TensorSpec{name, shape}` 목록), `intra_op_threads` (기본 1).
- 엔진은 N 입력 × N 출력 head 를 지원하고, `Init()` 을 여러 번 호출해 복수 모델을 등록한다. 같은 인스턴스를 다시 초기화하려면 `Reset()` 을 먼저 부른다.
- `input_buffer(m, i)` / `output_buffer(m, o)` 의 인덱스는 **`config` 선언 순서**다. 범위 밖이면 `nullptr`, 크기 접근자는 `0`.
- `Run()` · `RunModel(idx)` 는 IoBinding 경로이고, **RT 루프에는 `RunModels(indices, count)`** (직접 `Session::Run`) 를 쓴다. 모두 실패 시 예외 없이 `false`.
- 세션은 `ORT_SEQUENTIAL` · 단일 intra-op 스레드 · busy-spin 끔으로 만든다.

### 텐서를 이름으로 바인딩하는 이유

positional 바인딩은 같은 shape 의 두 텐서 (LSTM 의 `h_out`/`c_out`, fingertip F/T 모델의 `[1,3]` 두 head) 가 반대 순서로 export 되어도 **모든 shape 검사를 통과한 채 값이 조용히 뒤바뀐다.** 그래서 각 side (입력 / 출력) 는 다음 중 하나다.

| 선언 | 바인딩 | 비고 |
|---|---|---|
| 모든 텐서에 이름 | 이름 기준 | 뒤바뀜을 검출한다. `config` 순서가 모델 순서와 달라도 된다 |
| 모든 이름이 빈 문자열 | positional | 이름을 모르는 소비자용. 위 사각을 물려받는다 |
| 일부만 이름 | 거부 | `Init()` 이 실패한다 (바인딩이 모호하므로 추측하지 않는다) |

### 모델 검증

`Init()` 은 버퍼를 할당하기 전에 모델의 실제 입출력 arity·shape 를 `config` 와 대조하고, 불일치가 하나라도 있으면 **전량을 표로 담은** `std::runtime_error` 를 던진다. 입력·출력 arity 는 양쪽 모두 정확히 일치해야 하고, `config` 쪽 dim 은 static positive 여야 한다 (모델 쪽 dim `< 0` 은 dynamic). 규칙의 SSoT 는 [shape_report.hpp](include/rtc_inference/shape_report.hpp) 다.

### 스텁 (ORT 없이 빌드된 경우)

`HAS_ONNXRUNTIME` 이 정의되지 않으면 `Init()` 은 no-op, `Run*()` 은 `false`, 버퍼는 `nullptr`, 크기·`num_models()` 는 `0`, `is_initialized()` 는 `false` 다. 소비자는 `is_initialized()` 로 추론을 건너뛸 수 있다.

### 오프라인 모델 검사 (`rtc_inference_check`)

`.onnx` 가 무엇을 노출하는지 bring-up 없이 확인한다. ONNX Runtime 이 있을 때만 빌드된다.

```bash
# 모델이 노출하는 텐서 덤프
ros2 run rtc_inference rtc_inference_check policy.onnx

# 선언과 대조 (불일치 시 표를 찍고 exit 1)
ros2 run rtc_inference rtc_inference_check policy.onnx \
    --input obs:1x34 --output action:1x6

# 한쪽만 선언해도 된다 — 선언한 쪽만 판정하고 나머지는 덤프한다
ros2 run rtc_inference rtc_inference_check policy.onnx --input obs:1x34
```

이름 없는 텐서 (positional) 는 `:1x34` 처럼 이름을 비워 선언한다. 이 도구는 엔진과 같은 `CompareModelIo` 를 호출하므로 판정이 `Init()` 과 갈라지지 않는다. 컨트롤러 YAML 은 읽지 않는다 (그 스키마의 파서는 `rtc_controllers` 소유).

## 설정

빌드 시점 설정뿐이며 YAML 은 없다 (모델 선언은 소비자 컨트롤러의 YAML).

### ONNX Runtime 감지

[CMakeLists.txt](CMakeLists.txt) 가 `find_library` / `find_path` 로 탐색한다. 우선순위·`find_package(onnxruntime)` 를 쓰지 않는 이유·캐시 정책은 그 파일의 주석이 SSoT 다.

| 상황 | 동작 |
|---|---|
| 발견 | `HAS_ONNXRUNTIME` 정의 전파 + 라이브러리 링크. configure 가 `ORT_API_VERSION` 을 출력하고 1.18 미만이면 WARNING (IR 10 모델을 못 연다) |
| 미발견 | 스텁 엔진으로 빌드 (빌드는 실패하지 않는다) |

- 위치 지정: `-DRTC_ONNXRUNTIME_ROOT=<dir>` (`lib/` 와 `include/` 를 가진 루트). 평소에는 `setup_env.sh` 가 `CMAKE_PREFIX_PATH` 에 올린 `/opt/onnxruntime` 이 고른다.
- 다른 ORT prefix 가 앞에 실린 overlay·CI 이미지에서는 그쪽이 이긴다 — `RTC_ONNXRUNTIME_ROOT` 로 의도한 트리를 지정한다.

### 런타임 업그레이드

버전의 SSoT 는 `install.sh` 의 `ONNXRT_VERSION` 이다. 설치된 머신에서도 `install_onnxruntime` 이 `/opt/onnxruntime/VERSION_NUMBER` 를 pin 과 비교해 다르면 새 tarball 로 교체한다. 그 뒤 **ORT 를 링크하는 패키지를 재구성**해야 한다 (`colcon list --packages-above rtc_inference`).

- soname 이 버전마다 달라, 재링크되지 않은 바이너리는 `libonnxruntime.so.<옛버전>: cannot open shared object file` 로 죽는다.
- colcon 에 `--packages-above rtc_inference --cmake-force-configure` 를 준다 — plain colcon 의 전체 형태는 [repo_scripts/README.md](../repo_scripts/README.md) "Plain `colcon build` 호환성" 이 SSoT 다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| 추론이 항상 `false`, 버퍼가 `nullptr` | ORT 없이 스텁으로 빌드됨 | configure 로그의 `ONNX Runtime not found` 확인 → ORT 설치 또는 `-DRTC_ONNXRUNTIME_ROOT` |
| `Init()` 이 표와 함께 `runtime_error` | `.onnx` 의 텐서가 `config` 선언과 다름 | `rtc_inference_check` 로 덤프·대조해 YAML 수정 |
| IR 10 모델이 안 열림 / WARNING | 런타임이 1.18 미만 | `install.sh` pin 으로 업그레이드 후 재구성 |
| 실행 시 `libonnxruntime.so.… cannot open` | 런타임 업그레이드 뒤 재링크 안 됨 | 위 [런타임 업그레이드](#런타임-업그레이드) |

## 더 읽기

- 테스트: `colcon test --packages-select rtc_inference` — `test/` 의 파일 헤더·CMakeLists 주석이 설계를 설명한다. 이름 바인딩 픽스처 재생성: [test/data/generate_fixtures.py](test/data/generate_fixtures.py)
- RT-1 조건부 수용: [invariants.md](../agent_docs/invariants.md)

## 의존성

[package.xml](package.xml) · `rtc_base` · `libonnxruntime-dev` (선택) · C++20 · MIT ([LICENSE](../LICENSE))
