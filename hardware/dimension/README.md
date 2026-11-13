# 보드 기구 설계용 공식 치수 자료

수집일: 2026-11-13. 단위: mm. 대상은 센서 IC 자체가 아니라 **사용 중인 완성 보드**이다.
Adafruit는 PID 4754 STEMMA QT, SparkFun BNO086은 SEN-22857 Qwiic으로 사용자 확인을 받았다. XIAO S3는 기본형이며 Sense/Plus는 대상이 아니다.

## 핵심 치수

| 보드 | PCB 평면 외곽 / 공식 명목 크기 | 장착 및 배치 치수 | 높이 정보와 한계 |
|---|---|---|---|
| XIAO ESP32S3 | 21 × 17.8 (제조사 명목값) | 상세 외곽·패드 위치는 `xiao_esp32s3/dimensions_v1.1.dxf` | 공식 STEP 확보. 기본형의 조립 높이는 여기서 수치 확정하지 않음. 헤더·안테나 케이블은 별도 확인 |
| XIAO ESP32C3 | 21 × 17.8 (제조사 명목값) | top/bottom DXF 및 v1.2 상세 치수 DXF 확보 | 공식 링크의 3D 모델은 접근 차단(403). 높이 미확정 |
| Adafruit BNO085 PID 4754 | CAD: 25.40 × 22.86, 모서리 R2.54 | 4개 장착홀 CAD drill Ø2.50, 중심 간격 20.32 × 17.78 | 제품 페이지 표기 25.6 × 22.7 × 4.6. **CAD 명목 외곽과 제품 표기가 다름**. STEP/F3D/STL 확보 |
| SparkFun BNO086 SEN-22857 | CAD 좌표 기준 X=30.48, Y=25.40 | 2개 홀 Ø3.302, 중심 간격 25.40 | 공식 Eagle CAD/도면 확보. 조립 높이·PCB 두께는 확인된 자료에서 수치 확정하지 않음 |
| MyoWare 2 DEV-27924 | 37.57 × 35.90 | END–MID 전극 중심 거리 25.40; 이 두 중심선에서 REF 중심까지 수직 거리 23.62 | 스냅 단자 포함 전체 높이 미확정. 공식 2D 도면·제조사 가이드 확보. STEP/PCB CAD는 찾지 못함 |
| Teensy 4.1 DEV-16996 (Ethernet PHY + 헤더) | 60.96 × 17.78; **폭 공차 ±0.6** | 외측 핀 피치 2.54, 두 핀열 간격 15.24, PCB 두께 1.57 | PJRC 측면도: USB 상부 높이 2.5, 하부 돌출 0.6, 전방 돌출 0.7. **이 값에는 납땜 헤더 높이가 포함되지 않음** |

위 숫자는 도면 명목값, CAD 설계값, 제품 표기값을 구분했다. 소수점 자릿수가 실제 제조 공차를 뜻하지 않는다. 공차가 명시되지 않은 항목은 임의 공차를 추가하지 않았다.

## IMU 장착홀 좌표

기준: 각 Eagle 파일의 원래 XY 좌표계, 보드 상면 기준. 원점은 PCB 외곽 바운딩 박스의 좌하단이며, 단위는 mm이다. 위 표의 가로·세로를 바꿔 배치할 경우 좌표도 함께 회전해야 한다.

| 보드 | 홀 | X | Y | CAD 드릴 지름 |
|---|---|---:|---:|---:|
| Adafruit BNO085 | U$19 | 2.54 | 2.54 | 2.50 |
| Adafruit BNO085 | U$21 | 22.86 | 2.54 | 2.50 |
| Adafruit BNO085 | U$1 | 2.54 | 20.32 | 2.50 |
| Adafruit BNO085 | U$17 | 22.86 | 20.32 | 2.50 |
| SparkFun BNO086 | JP3 | 2.54 | 22.86 | 3.302 |
| SparkFun BNO086 | JP4 | 27.94 | 22.86 | 3.302 |

Adafruit 홀은 plated pad이므로 Ø2.50은 CAD drill 값이다. 완제품의 보증된 최종 내경이나 나사 여유치로 해석하지 않는다. SparkFun 홀은 Eagle STAND-OFF 패키지의 hole 값이다. `pcb_geometry.json`에 외곽 선, 홀 정보, 전체 부품 배치 원점을 원본에서 추출해 저장했다. 부품 원점은 부품의 외곽이나 중심을 항상 뜻하지 않으며, 회전·패키지 로컬 좌표와 함께 사용해야 한다.

## 파일 안내

- `xiao_esp32s3/`: 공식 DXF v1.1, 3D 모델 ZIP 및 압축 해제된 STEP/미리보기.
- `xiao_esp32c3/`: 상·하면 DXF ZIP, v1.2 상세 치수 ZIP, KiCad v1.3 ZIP 및 압축 해제본.
- `adafruit_bno085/`: 공식 fab print, Eagle BRD/SCH, STEP/F3D/STL, 제조사 README와 라이선스, 추출한 기구 좌표.
- `sparkfun_bno086/`: 치수 PNG, Eagle v1.0 ZIP/압축 해제본, 라이선스, 추출한 기구 좌표.
- `myoware2/`: 공식 치수 PNG와 Advancer Technologies의 Advanced Guide PDF.
- `teensy41/`: PJRC 공식 치수도 PNG와 상·하면 치수 참고 사진. 사진의 자를 기준으로 치수를 추정하지 않았다.
- `sources.json`: 각 원본의 다운로드 URL, 연결된 공식 페이지, 수집일, 파일 크기, SHA-256, 미확보 항목.
- `dimensions.csv`: 핵심 치수와 근거를 담은 UTF-8 표. 빈 수치 필드는 미확정이며 0을 뜻하지 않는다.

## 원본 간 불일치 및 버전 주의

1. **XIAO ESP32S3:** 공식 기본형 페이지의 “KiCad Project” 링크 파일명은 v1.4이지만 ZIP 내부는 `XIAO ESP32S3 Sense_v1.5`이다. 이 자료는 `reference_mismatch/`로 분리했다. 기본형의 확정 설계 원본으로 사용하지 않는다. DXF는 v1.1이고 3D ZIP 내부 STEP 이름은 `XIAO-ESP32S3 v2.step`이다. 세 파일의 개정이 동일하다고 가정하지 않는다.
2. **Adafruit BNO085:** fab print/Eagle의 명목 외곽은 25.40 × 22.86이고 판매 페이지는 25.6 × 22.7 × 4.6이다. 4.6을 PCB 두께로 사용하면 안 된다. 홀 위치에는 PCB CAD를 사용하고 밀착 하우징의 실제 외곽/높이는 보유 보드와 대조한다.
3. **MyoWare:** 현재 SparkFun 제품 페이지 SKU DEV-27924가 연결한 도면을 저장했다. 제조사 Advanced Guide는 2022 경로의 MyoWare v2 공통 자료이므로 현재 SKU의 모든 조립 부품과 개정이 동일하다고 단정하지 않는다. 전극 스냅은 나사 장착홀이 아니다. 삼각형의 정확한 곡선 외곽/스냅 높이를 단순 사각형 치수로 재구성하지 않는다.
4. **Teensy:** DEV-16996은 헤더가 납땜된 제품이다. Ethernet PHY가 있다는 뜻이지 RJ45 잭 전체가 실장되어 있다는 뜻은 아니다. PJRC는 2023년 3월 이후 SD 소켓 변경도 명시하므로 SD 삽입부의 상세 형상은 보유 보드와 비교한다. 도면상 USB 포함 길이 61.66 (=60.96+0.70)은 계산값이며, 케이블을 꽂은 전체 길이가 아니다.

## 3D 설계에 적용할 때

1. 가능하면 STEP를 기준 형상으로 가져오고 DXF/BRD의 외곽 및 홀 좌표와 교차 확인한다. STL은 삼각형 메시이므로 정밀 치수 수정에는 STEP를 우선한다.
2. 보드 리비전, 헤더 방향/길이, 납땜 돌출, Qwiic/STEMMA 플러그, USB 케이블 몰딩, 안테나 케이블, MyoWare 전극/실드 적층을 실제 구성과 대조한다.
3. 높이 미확정 항목과 케이블 삽입 공간을 확인한 후 하우징을 확정한다. 출력 여유·공차는 프린터/재료/조립 방식에 따라 따로 정해야 하며 제조사 명목 치수와 섞지 않는다.

## 공식 출처

- [Seeed XIAO ESP32S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
- [Seeed XIAO ESP32C3](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/)
- [Seeed가 연결한 C3 3D 모델: GrabCAD](https://grabcad.com/library/seeed-studio-xiao-esp32-c3-1) - HTTP 403으로 미확보.
- [Adafruit BNO085 Downloads](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/downloads), [제품 페이지](https://www.adafruit.com/product/4754), [PCB 저장소](https://github.com/adafruit/Adafruit-BNO08x-PCB), [CAD 저장소](https://github.com/adafruit/Adafruit_CAD_Parts/tree/main/4754%20BNO085%20STEMMA%20QT)
- [SparkFun BNO086 공식 가이드](https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/single_page/)
- [SparkFun MyoWare DEV-27924](https://www.sparkfun.com/myoware-2-muscle-sensor.html), [MyoWare 제조사 지원](https://myoware.com/support/)
- [PJRC Teensy 4.1 공식 치수](https://www.pjrc.com/store/teensy41.html), [SparkFun DEV-16996 제품 확인](https://www.sparkfun.com/teensy-4-1-headers.html)

원본 파일의 저작권·라이선스는 각 제조사에 있다. SparkFun/Adafruit 라이선스 파일을 함께 보관했다. PJRC가 연결한 사용자 제작 3D 모델은 제조사 원본이 아니므로 포함하지 않았다. 이 모음은 보드 치수 자료이며, 공차 검증이 끝난 하우징 제작 도면은 아니다.
