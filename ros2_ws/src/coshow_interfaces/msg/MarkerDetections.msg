# 드론 1대의 한 프레임 검출 결과
std_msgs/Header header                # stamp = 검출 완료 시각
string drone                          # "cf_a" 등
geometry_msgs/PoseStamped drone_pose  # 검출 시점의 최신 pose
MarkerDetection[] markers             # 검출된 마커 목록 (0개 이상)
