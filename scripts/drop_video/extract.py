"""Decode a 240 fps slow-mo video at 960x540 gray and store three ROI crops for every frame."""
import json, subprocess, sys
import numpy as np, imageio_ffmpeg
src, out = sys.argv[1], sys.argv[2]
W, H = 960, 540
ROIS = {  # x0, x1, y0, y1 in 960x540 coordinates
    'horn':    (690, 790, 135, 275),
    'slider':  (575, 690, 125, 225),
    'payload': (440, 600, 250, 335),
}
n_est = int(sys.argv[3])
if len(sys.argv) > 4:
    ROIS = {k: tuple(v) for k, v in json.loads(sys.argv[4]).items()}
mm = {k: np.lib.format.open_memmap(f'{out}_{k}.npy', mode='w+', dtype=np.uint8,
                                   shape=(n_est, y1 - y0, x1 - x0)) for k, (x0, x1, y0, y1) in ROIS.items()}
cmd = [imageio_ffmpeg.get_ffmpeg_exe(), '-v', 'error', '-threads', '4', '-i', src,
       '-vf', f'scale={W}:{H}', '-pix_fmt', 'gray', '-f', 'rawvideo', '-']
p = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=W * H * 8)
n = 0
while True:
    buf = p.stdout.read(W * H)
    if len(buf) < W * H:
        break
    fr = np.frombuffer(buf, np.uint8).reshape(H, W)
    if n < n_est:
        for k, (x0, x1, y0, y1) in ROIS.items():
            mm[k][n] = fr[y0:y1, x0:x1]
    n += 1
    if n % 5000 == 0:
        print(n, flush=True)
for m in mm.values():
    m.flush()
json.dump({'frames': n, 'rois': ROIS}, open(out + '_meta.json', 'w'))
print('done', n)
