# 마커 하나의 검출 정보
int32 id          # 마커 ID
float32 cx        # 화면 중심 x (픽셀)
float32 cy        # 화면 중심 y (픽셀)
float32 size_px   # 화면상 한 변 크기 (픽셀)
float32 world_x   # 역투영한 마커 실제 지면 좌표 x (map), 미계산시 0
float32 world_y   # 역투영한 마커 실제 지면 좌표 y (map), 미계산시 0
