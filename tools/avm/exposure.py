"""Replay ARM gain updates from actual RTL overlap sums or fixture statistics.

python tools/avm/exposure.py --statistics sim/data/small/rtl_statistics.json --output feedback.json
Input: list of {cameras:[a,b], count:N, sums:[[R,G,B],[R,G,B]]}, or {statistics:list}.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from model import bounded_log_gains


def replay(statistics, updates=60, previous=None):
    means, counts = {}, {}
    for item in statistics:
        pair = tuple(item['cameras'])
        count = int(item['count'])
        if count:
            means[pair] = np.asarray(item['sums'], float)/count
            counts[pair] = count
    def residual(gains):
        return {f'{a}-{b}': (np.abs(m[0]*gains[a]-m[1]*gains[b])/(.5*(m[0]*gains[a]+m[1]*gains[b]))).tolist()
                for (a,b),m in means.items()}
    gains = np.ones((4,3)) if previous is None else np.array(previous,float)
    before = residual(gains)
    held=[]
    history=[]
    for _ in range(updates):
        gains,held=bounded_log_gains(means,counts,gains)
        history.append(gains.tolist())
    qgains=np.floor(gains*4096+.5).astype(int)
    after=residual(qgains/4096)
    mean_before=float(np.mean(list(before.values()))) if before else 0.
    mean_after=float(np.mean(list(after.values()))) if after else 0.
    return dict(update_count=updates, update_frequency_hz=5, ema_log_coefficient=.1,
                gains_q4_12=qgains.tolist(), gains=gains.tolist(), held_channels=held,
                residual_before=before, residual_after=after,
                mean_relative_residual_before=mean_before,mean_relative_residual_after=mean_after,
                max_relative_residual_after=float(np.max(list(after.values()))) if after else 0., history=history)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--statistics',type=Path,required=True)
    parser.add_argument('--updates',type=int,default=60)
    parser.add_argument('--previous',type=Path,help='Previous feedback.json; uses its unquantized gains state')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    payload=json.loads(args.statistics.read_text(encoding='utf-8-sig'))
    stats=payload['statistics'] if isinstance(payload,dict) else payload
    previous=json.loads(args.previous.read_text())['gains'] if args.previous else None
    result=replay(stats,args.updates,previous)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='history'},indent=2))


if __name__=='__main__':
    main()
