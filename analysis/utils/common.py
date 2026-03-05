import re
from pathlib import Path
from typing import Any, Dict, Union, Optional

import numpy as np
import pandas as pd


def read_csv_logging(path: Union[str, Path]) -> Dict[str, Any]:
    path = Path(path)

    df = pd.read_csv(path, sep=";", engine="python")
    df.columns = [str(c).strip() for c in df.columns]

    col_to_arr: Dict[str, np.ndarray] = {}
    for c in df.columns:
        col_to_arr[c] = df[c].to_numpy()

    pat = re.compile(r"^(?P<base>.+)_(?P<idx>\d+)$")

    groups: Dict[str, Dict[int, str]] = {}
    for c in df.columns:
        m = pat.match(c)
        if not m:
            continue
        base = m.group("base")
        idx = int(m.group("idx"))
        if base not in groups:
            groups[base] = {}
        groups[base][idx] = c

    out: Dict[str, Any] = {}
    used = set()

    for base, idx_map in groups.items():
        idxs = sorted(idx_map.keys())
        if not idxs:
            continue
        if idxs != list(range(idxs[0], idxs[-1] + 1)):
            continue
        cols = [idx_map[i] for i in idxs]
        mat = df[cols].to_numpy(dtype=float, copy=False)
        out[base] = mat
        used.update(cols)

    for c in df.columns:
        if c in used:
            continue
        out[c] = col_to_arr[c]

    if "time" in out:
        t = np.asarray(out["time"], dtype=float)
        if t.size > 0:
            out["time"] = t - t[0]

    return out