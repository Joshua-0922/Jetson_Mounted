# drop_video

240 fps 슬로모션 영상에서 투하 지연(서보 움직임 시작 → 페이로드 분리)을 재는 스크립트. 결과와 해석은 `docs/sim2real_params/`.

```bash
pip install --user opencv-python-headless imageio-ffmpeg scipy matplotlib
# 1) 프레임마다 관심 영역(서보 혼, 래치 슬라이더, 페이로드)을 잘라 저장. 좌표는 960x540 기준, 영상마다 구도에 맞게 지정
python3 extract.py VIDEO.mp4 v1 <프레임 수> '{"horn":[690,790,135,275],"slider":[575,690,125,225],"payload":[440,600,250,335]}'
python3 extract.py VIDEO.mp4 c1 <프레임 수> '{"pcol":[455,585,195,425]}'     # 페이로드 세로 영역 (낙하 추적용)
# 2) 서보 동작 구간 찾기 (열림/닫힘) -> v1_events.json
python3 detect.py v1
# 3) 회차별 서보 시작·분리 시각 (낙하 포물선 역외삽) -> drop_delay_results.json   (VIDEOS 표에서 영상·회차 번호 지정)
python3 analyze_drop.py
# 4) 분포 적합, 상관, 그래프 -> drop_delay_stats.json, drop_delay.png   (OUTLIER, 명령 로그 경로 확인)
python3 stats_drop.py
```
