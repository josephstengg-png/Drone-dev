#!/usr/bin/env python3
"""Per armed PX4 log (.ulg): how long it was armed, when `offboard_control_signal_lost` and `gcs_connection_lost` first fired
(seconds after arming) and the longest gap between OFFBOARD_CONTROL_MODE messages reaching PX4. Read-only.

    .venv/bin/python tools/ulog_failsafes.py [DIR_OF_ULGS]     # default fc-backups/2026-11-05/ulg

Written 5 Nov 2026 for Experiment 3: it showed the failsafe in every armed run was the offboard signal, not the GCS link.
"""
import sys, glob, numpy as np
from pyulog import ULog
DIR = sys.argv[1] if len(sys.argv) > 1 else '/home/joseph/drone-dev/fc-backups/2026-11-05/ulg'
for f in sorted(glob.glob(DIR + '/*.ulg')):
    u = ULog(f); D = {d.name: d.data for d in u.data_list}
    vs = D.get('vehicle_status'); ff = D.get('failsafe_flags'); oc = D.get('offboard_control_mode')
    if vs is None: print(f[-22:], 'no vehicle_status'); continue
    arm = vs['arming_state']; t = vs['timestamp']/1e6
    ia = np.where(arm == 2)[0]
    if not len(ia): print(f.split('/')[-1][:8], 'never armed', 'gcsflag' if ff is not None and 'gcs_connection_lost' in ff else ''); continue
    ta = t[ia[0]]; idis = np.where((arm == 1) & (t > ta))[0]; td = t[idis[0]] if len(idis) else None
    out = f"{f.split('/')[-1][:8]} armed {ta:.0f}s armed_for {td-ta:.1f}s" if td else f"{f.split('/')[-1][:8]} armed {ta:.0f}s still"
    if ff is not None:
        ft = ff['timestamp']/1e6
        for k in ('offboard_control_signal_lost','gcs_connection_lost','manual_control_signal_lost'):
            if k in ff:
                idx = np.where((ff[k] != 0) & (ft >= ta - 0.5))[0]
                out += f" | {k[:12]} first {ft[idx[0]]-ta:.1f}s n_on={int(np.sum(np.diff(np.r_[0, (ff[k]!=0).astype(int)])==1))}" if len(idx) else f" | {k[:12]} none"
    if oc is not None:
        ot = oc['timestamp']/1e6; sel = ot >= ta
        g = np.diff(ot[sel]) if sel.sum() > 2 else np.array([])
        out += f" | offb_ctl_mode msgs {int(sel.sum())} maxgap {g.max():.2f}s" if len(g) else " | offb_ctl_mode none"
    print(out)
