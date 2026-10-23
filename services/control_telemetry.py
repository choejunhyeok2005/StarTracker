"""Streaming CSV plus dependency-free, offline HTML plots."""
import csv
import html
import json
from pathlib import Path


FIELDS = ("time_s", "target_alt", "actual_alt", "error_alt", "target_az",
          "actual_az", "error_az", "speed_alt", "speed_az", "steps_alt", "steps_az",
          "accel_alt", "gyro_rate", "gyro_bias", "innovation_deg", "angle_variance", "kalman_gain")


class ControlTelemetry:
    def __init__(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()

    def record(self, **row):
        self.writer.writerow(row)
        self.file.flush()

    def close(self):
        self.file.close()


def render_report(csv_path, html_path, label=None, az_is_estimate=False):
    with open(csv_path, encoding="utf-8", newline="") as source:
        rows = [{k: float(v) for k, v in row.items() if v != ""} for row in csv.DictReader(source)]
    if not rows:
        raise ValueError("No control samples to plot.")
    data = json.dumps(rows, allow_nan=False)
    title = html.escape(label or Path(csv_path).name)
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8">
<title>StarTracker Control Report</title><style>
body{background:#101827;color:#e5edf9;font:16px system-ui;margin:32px auto;max-width:1100px}
canvas{width:100%;height:260px;background:#172339;border-radius:12px}h2{font-size:18px}
p{color:#aabcce}code{color:#6fe4cc}</style>
<h1>StarTracker · 폐루프 위치제어</h1><p>__TITLE__ · ALT = EL · 각도 ° / 시간 s</p>
<p>하늘색: 목표각 · 초록색: 실제각. AZ는 0–360° 표시이므로 경계에서 선이 끊겨 보일 수 있습니다.</p>
<h2>ALT 목표 / 실제</h2><canvas id="alt" width="1100" height="260"></canvas>
<h2>AZ 목표 / 실제</h2><canvas id="az" width="1100" height="260"></canvas>
<h2>위치 오차 (ALT: 하늘색 / AZ: 초록색)</h2><canvas id="error" width="1100" height="260"></canvas>
<p id="metrics"></p><script>
const rows=__DATA__;
function draw(id, keys){const c=document.getElementById(id),ctx=c.getContext('2d');
let lo=Infinity,hi=-Infinity;for(const r of rows)for(const k of keys){lo=Math.min(lo,r[k]);hi=Math.max(hi,r[k]);}
const pad=Math.max((hi-lo)*.1,.01);lo-=pad;hi+=pad;
const t0=rows[0].time_s,t1=rows.at(-1).time_s;
ctx.font='13px system-ui';ctx.strokeStyle='#34445c';ctx.fillStyle='#aabcce';
for(let i=0;i<=4;i++){let y=20+i*50;ctx.beginPath();ctx.moveTo(75,y);ctx.lineTo(1080,y);ctx.stroke();ctx.fillText((hi-(hi-lo)*i/4).toFixed(3),4,y+5);}
for(let i=0;i<=5;i++)ctx.fillText(((t1-t0)*i/5).toFixed(1)+'s',75+195*i,248);
keys.forEach((k,n)=>{ctx.strokeStyle=['#75bbff','#6fe4cc'][n];ctx.lineWidth=2;ctx.beginPath();rows.forEach((r,i)=>{let x=75+(r.time_s-t0)/Math.max(t1-t0,.001)*1005,y=20+(hi-r[k])/(hi-lo)*200;i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();});}
draw('alt',['target_alt','actual_alt']);draw('az',['target_az','actual_az']);draw('error',['error_alt','error_az']);
function extra(id,title,keys){if(!rows.every(r=>keys.every(k=>Number.isFinite(r[k]))))return;
const h=document.createElement('h2');h.textContent=title;document.body.append(h);
const c=document.createElement('canvas');c.id=id;c.width=1100;c.height=260;document.body.append(c);draw(id,keys);}
extra('filter','EL 가속도 각도 / 필터 추정각',['accel_alt','actual_alt']);
extra('bias','자이로 측정 속도 / 추정 바이어스 (°/s)',['gyro_rate','gyro_bias']);
extra('innovation','칼만 잔차 (°)',['innovation_deg']);
document.getElementById('metrics').textContent=['alt','az'].map(a=>a.toUpperCase()+' RMS 오차: '+Math.sqrt(rows.reduce((s,r)=>s+r['error_'+a]**2,0)/rows.length).toFixed(4)+'°').join(' · ');
</script></html>'''
    if az_is_estimate:
        page = page.replace('AZ 목표 / 실제', 'AZ 목표 / 스텝 추정 (센서 측정 아님)')
        page = page.replace('하늘색: 목표각 · 초록색: 실제각.',
                            '하늘색: 목표각 · 초록색: EL은 IMU 측정각, AZ는 스텝 추정각.')
    Path(html_path).write_text(page.replace('__TITLE__', title).replace('__DATA__', data), encoding="utf-8")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    parser.add_argument("html")
    parser.add_argument("--imu", action="store_true", help="Label AZ as step-estimated")
    args = parser.parse_args()
    render_report(args.csv, args.html, az_is_estimate=args.imu)
