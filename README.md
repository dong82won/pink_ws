PinkLAB STUDY


# 2-3. 토픽 구둑하여 메시지 출력해보기 (OK)

# 2-4. 내가 정의한 메시지 사용해보기 (OK)

# 2-5. 하나의 노드에서 두 개 이상의 토픽 다루기 1 (OK)

# 2-6. 하나의 노드에서 두 개 이상의 토픽 다루기 2 (OK)

# 2-7. 서비스 메시지 정의 만들기 (OK)

# 2-8. 서비스 서버 만들기 (OK)


# 2-9. 서비스 서버 만들기 응용편 (OK)

이 코드는 **"중간 관리자"** 역할을 하는 노드입니다.

이 코드가 헷갈리는 이유는 하나의 노드가 **서비스 요청을 받는 역할(서버)**
과 **다른 노드에게 요청을 보내는 역할(클라이언트)** 을 동시에 하고 있기 때문입니다.

쉽게 이해하실 수 있도록 **그림**과 **비유**를 들어 설명해 드리겠습니다.

### 1\. 전체적인 흐름 (비유: 식당의 웨이터)

이 노드(`MultiSpawning`)는 **웨이터**와 같습니다.

1.  **손님(사용자)** 이 웨이터에게 "주문(MultiSpawn 요청)"을 합니다. (**서버 역할**)
2.  **웨이터**는 주문을 받으면 주방장(거북이 화면, Turtlesim)에게 "요리해라(이동해라)"라고 명령을 전달합니다. (**클라이언트 역할**)

즉, **"내가 요청을 받으면(`multi_spawn`), 다른 녀석(`turtle1`)을 움직이게 하겠다"** 는 코드입니다.

### 2\. 코드 한 줄 한 줄 뜯어보기

#### 1\) 초기화 부분 (`__init__`) : 준비 단계

여기서는 웨이터가 영업 준비를 합니다. 두 가지를 준비합니다.

```python
def __init__(self):
    super().__init__('multi_spawn')
    
    # 1. 서버 준비 (주문 받을 준비)
    # 'multi_spawn'이라는 이름으로 요청이 들어오면 self.service_callback 함수를 실행해라!
    self.srv = self.create_service(MultiSpawn, 'multi_spawn', self.service_callback)

    # 2. 클라이언트 준비 (주방에 전달할 준비)
    # 나는 'turtle1/teleport_absolute'라는 서비스(거북이 이동)를 이용할 것이다!
    self.teleport = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
    
    # 요청 보낼 쪽지(메시지)를 미리 만들어 둠
    self.req_teleport = TeleportAbsolute.Request()
```

  * **`create_service`**: 누군가 나를 부를 수 있게 귀를 열어두는 것입니다.
  * **`create_client`**: 내가 누군가를 부르기 위해 전화기를 준비하는 것입니다.

-----

#### 2\) 콜백 함수 (`service_callback`) : 실제 행동

누군가 `multi_spawn` 서비스를 요청했을 때 **실제로 실행되는 함수**입니다.

```python
def service_callback(self, request, response):
    # 1. 거북이를 어디로 보낼지 내용을 채웁니다.
    self.req_teleport.x = 1.0  # X좌표 1.0
    self.req_teleport.y = 1.0  # Y좌표 1.0
    
    # 2. 거북이(Turtlesim)에게 실제로 명령을 보냅니다 (비동기 호출)
    # "거북아, (1.0, 1.0)으로 순간이동 해!"
    self.teleport.call_async(self.req_teleport)
 
    # 3. 나에게 요청한 사람에게 "알겠다"고 응답을 돌려줍니다.
    return response
```

  * **핵심**: 이 함수 안에서 `self.teleport.call_async`를 쓴 것이 가장 중요합니다. 내 서비스를 수행하는 과정에서 **또 다른 서비스를 호출**한 것입니다.

### 3\. 요약

이 코드는 다음과 같이 동작합니다.

1.  사용자가 터미널에서 `multi_spawn` 서비스를 호출합니다.
2.  이 노드의 `service_callback` 함수가 발동됩니다.
3.  이 함수는 내부적으로 `turtle1/teleport_absolute` 클라이언트를 사용하여 거북이에게 `(1.0, 1.0)`으로 가라고 명령합니다.
4.  결과적으로, **사용자는 `MultiSpawn`을 호출했는데 화면 속 거북이가 움직이게 됩니다.**

---
`TeleportAbsolute.Request()` 문법이 낯선 것은 지극히 정상입니다. 파이썬 문법이라기보다는 **ROS2가 내부적으로 파일을 변환하는 방식**을 알아야 이해할 수 있는 부분이기 때문입니다.
---

한마디로 정의하면 **"빈 신청서(주문서) 한 장을 뽑아 드는 행위"**
입니다. 왜 이런 문법이 나왔는지 3단계로 쪼개서 설명해 드리겠습니다.


### 1\. 원본: `.srv` 파일 (서비스 정의 파일)

ROS2에서 서비스는 `.srv`라는 텍스트 파일로 정의됩니다. `turtlesim` 패키지 안에 있는 `TeleportAbsolute.srv` 파일을 열어보면 실제로 이렇게 생겼습니다.

```text
# TeleportAbsolute.srv 파일 내용

float32 x      # (1) 요청할 때 보낼 데이터 (Request)
float32 y
float32 theta
---            # (구분선)
               # (2) 응답으로 받을 데이터 (Response) - 여긴 비어있음
```

  * **윗부분**: 요청(Request) - "어디로 갈래?" (x, y, 각도)
  * **아랫부분**: 응답(Response) - "잘 도착했음" (보통 비어있거나 성공 여부)

-----

### 2\. 변환: 빌드(Build) 후 파이썬의 모습

`colcon build`를 통해 컴파일을 하면, ROS2는 위 텍스트 파일을 파이썬 클래스로 **자동 변환**해줍니다. 이때 **큰 봉투(클래스)** 하나에 \*\*두 개의 작은 쪽지(내부 클래스)\*\*를 넣어줍니다.

개발자가 `from turtlesim.srv import TeleportAbsolute`라고 썼을 때, 가져오는 `TeleportAbsolute`는 사실 아래와 같은 구조를 가진 **큰 컨테이너**입니다.

```python
# (개념적인 구조입니다)
class TeleportAbsolute:
    
    # 위쪽(.srv의 윗부분)을 담당하는 클래스
    class Request:
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    # 아래쪽(.srv의 아랫부분)을 담당하는 클래스
    class Response:
        pass
```

-----

### 3\. 문법 해석: `TeleportAbsolute.Request()`

이제 질문하신 코드를 다시 보겠습니다.

```python
self.req_teleport = TeleportAbsolute.Request()
```

이 코드는 다음과 같이 해석됩니다.

1.  **`TeleportAbsolute`**: 큰 서비스 꾸러미를 엽니다.
2.  **`.Request`**: 그 안에 있는 "요청용 설계도(입력 양식)"를 선택합니다.
3.  **`()`**: **"이 양식대로 빈 종이 한 장 주세요\!"** 하고 객체(Instance)를 생성합니다.

만약 이 줄이 없다면, 데이터를 담을 그릇 자체가 없는 상태입니다. 그래서 **빈 그릇(req\_teleport)을 먼저 만들고**, 그 뒤에 데이터를 채워 넣는 것입니다.

```python
# 1. 빈 신청서 한 장 뽑기
self.req_teleport = TeleportAbsolute.Request()

# 2. 신청서에 내용 적기 (나중에 콜백 함수 안에서 하는 일)
self.req_teleport.x = 1.0
self.req_teleport.y = 1.0
```

### 요약

  * **`TeleportAbsolute`**: 서비스 전체 이름 (큰 틀)
  * **`TeleportAbsolute.Request`**: 그 서비스 중 "보내는 데이터"에 대한 정의 (신청서 양식)
  * **`()`**: 실제로 데이터를 채워 넣을 수 있게 **빈 신청서 한 장을 생성**하는 것.

###



# 2-12 다수의 서비스 클라이언트 구현하기 (OK)
